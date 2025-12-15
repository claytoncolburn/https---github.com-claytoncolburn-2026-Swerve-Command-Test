// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configs;

public class SwerveModule {
  private final Integer m_turningCanId;
  private final SparkFlex m_drivingFlex;
  private final SparkMax m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final RelativeEncoder m_turningRelativeEncoder;
//   private final AbsoluteEncoder m_turningEncoder;
  private final AnalogInput m_turningAbsoluteEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private Rotation2d m_chassisAngularOffset;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public SwerveModule(int drivingCANId, int turningCANId, int absEncoder, Rotation2d chassisAngularOffset) {
    m_turningCanId = turningCANId;
    m_drivingFlex = new SparkFlex(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingFlex.getEncoder();
    m_turningRelativeEncoder = m_turningSpark.getEncoder();
    // m_turningEncoder = m_turningSpark.getAbsoluteEncoder();
    m_turningAbsoluteEncoder = new AnalogInput(absEncoder);

    m_drivingClosedLoopController = m_drivingFlex.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_drivingFlex.configure(Configs.SwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSpark.configure(Configs.SwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = getAbsoluteEncoder2D();
    // m_desiredState.angle = Rotation2d.fromDegrees(180.0);
    m_turningRelativeEncoder.setPosition(m_desiredState.angle.getDegrees());
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(), getAbsoluteEncoder2D());
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(m_drivingEncoder.getPosition(), getAbsoluteEncoder2D());
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean stopMotors) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle;

    //Rotation2d absRotation2d = getAbsoluteEncoder2D();
    //
    // double currentTurningAngle = m_turningRelativeEncoder.getPosition();
    // if (currentTurningAngle > 180 || currentTurningAngle <= -180) {
    //     currentTurningAngle = MathUtil.inputModulus(currentTurningAngle, -180, 180);
    //     m_turningRelativeEncoder.setPosition(currentTurningAngle);
    // }

    // TODO: Should we periodically re-calibrate the relative encoder with the absolute
    // if (Math.abs(currentTurningAngle - absRotation2d.getDegrees()) > 1.0) {
    //     m_turningRelativeEncoder.setPosition(absRotation2d.getDegrees());
    // }

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(Rotation2d.fromDegrees(m_turningRelativeEncoder.getPosition()));

    if (stopMotors) {
        m_drivingFlex.set(0);
        m_turningSpark.set(0);
    } else {
    // Command driving and turning SPARKS towards their respective setpoints.
        m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
        m_turningClosedLoopController.setReference(correctedDesiredState.angle.getDegrees(), ControlType.kPosition);
    }

    SmartDashboard.putNumber("desired" + this.m_turningCanId.toString(), desiredState.angle.getDegrees());
    SmartDashboard.putNumber("corrected" + this.m_turningCanId.toString(), correctedDesiredState.angle.getDegrees());
    SmartDashboard.putNumber("absolute" + this.m_turningCanId.toString(), getAbsoluteEncoder2D().getDegrees());
    SmartDashboard.putNumber("voltage" + this.m_turningCanId.toString(), m_turningAbsoluteEncoder.getVoltage());
    SmartDashboard.putNumber("5v" + this.m_turningCanId.toString(), RobotController.getVoltage5V());
    SmartDashboard.putNumber("relative" + this.m_turningCanId.toString(), m_turningRelativeEncoder.getPosition());


    m_desiredState = desiredState;
  }

  private Rotation2d getAbsoluteEncoder2D() {
    double encoder01 = 1.0 - (m_turningAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V());
    return Rotation2d.fromDegrees(encoder01 * 360).plus(m_chassisAngularOffset).minus(Rotation2d.k180deg);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}