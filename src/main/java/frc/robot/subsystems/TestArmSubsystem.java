package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.jni.REVLibJNI;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class TestArmSubsystem extends SubsystemBase {
    
  private final SparkMax m_testArm;
  private final RelativeEncoder m_testArmEncoder;
  private final SparkClosedLoopController m_testArmClosedLoopController;

  public TestArmSubsystem() {
    m_testArm = new SparkMax(9, MotorType.kBrushless);
    m_testArmEncoder = m_testArm.getEncoder();

    m_testArmClosedLoopController = m_testArm.getClosedLoopController();

    m_testArm.configure(Configs.SwerveModule.testArmConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void drive(double value) {
    // m_testArm.set(value);
    
    m_testArmClosedLoopController.setReference(value, ControlType.kPosition);
  }

  public void stopMotor() {
    m_testArm.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("testArm Encoder", m_testArmEncoder.getPosition());
  }



}
