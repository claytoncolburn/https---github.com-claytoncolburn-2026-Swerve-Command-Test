// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.TestArmSubsystem;
// Old DriveSubsystem (WPILib-based, no vision) - REPLACED BY YAGSL
// import frc.robot.subsystems.drive.DriveSubsystem;
// YAGSL + Vision subsystems
import frc.robot.subsystems.drive.YAGSLDriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import swervelib.SwerveInputStream;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  // NOTE: Only ONE drive subsystem should be active at a time (they control the same hardware)

  // Old DriveSubsystem (WPILib-based, no vision) - REPLACED BY YAGSL
  // private final DriveSubsystem m_robotDrive;

  // YAGSL + Vision subsystems
  // Current: Single Limelight configuration
  private final VisionSubsystem m_visionSubsystem;
  private final YAGSLDriveSubsystem m_robotDrive;

  // Alternative: Multiple Limelights for redundancy/wider coverage (uncomment and modify as needed)
  //
  // Two Limelights (front and back):
  // private final VisionSubsystem m_visionFront;
  // private final VisionSubsystem m_visionBack;
  // private final YAGSLDriveSubsystem m_robotDrive;
  //
  // Three Limelights (front, left, right):
  // private final VisionSubsystem m_visionFront;
  // private final VisionSubsystem m_visionLeft;
  // private final VisionSubsystem m_visionRight;
  // private final YAGSLDriveSubsystem m_robotDrive;
  //
  // Four Limelights (front, back, left, right):
  // private final VisionSubsystem m_visionFront;
  // private final VisionSubsystem m_visionBack;
  // private final VisionSubsystem m_visionLeft;
  // private final VisionSubsystem m_visionRight;
  // private final YAGSLDriveSubsystem m_robotDrive;

  private final TestArmSubsystem m_testArm;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Initialize drive subsystem
    // Old: WPILib-based DriveSubsystem (REPLACED BY YAGSL)
    // m_robotDrive = new DriveSubsystem();

    // YAGSL + Vision initialization
    // Current: Single Limelight configuration
    m_visionSubsystem = new VisionSubsystem("limelight");
    m_robotDrive = new YAGSLDriveSubsystem(m_visionSubsystem);

    // Alternative: Multiple Limelights for redundancy/wider coverage (uncomment and modify as needed)
    //
    // Two Limelights (front and back):
    // m_visionFront = new VisionSubsystem("limelight-front");
    // m_visionBack = new VisionSubsystem("limelight-back");
    // m_robotDrive = new YAGSLDriveSubsystem(m_visionFront, m_visionBack);
    //
    // Three Limelights (front, left, right):
    // m_visionFront = new VisionSubsystem("limelight-front");
    // m_visionLeft = new VisionSubsystem("limelight-left");
    // m_visionRight = new VisionSubsystem("limelight-right");
    // m_robotDrive = new YAGSLDriveSubsystem(m_visionFront, m_visionLeft, m_visionRight);
    //
    // Four Limelights (front, back, left, right):
    // m_visionFront = new VisionSubsystem("limelight-front");
    // m_visionBack = new VisionSubsystem("limelight-back");
    // m_visionLeft = new VisionSubsystem("limelight-left");
    // m_visionRight = new VisionSubsystem("limelight-right");
    // m_robotDrive = new YAGSLDriveSubsystem(m_visionFront, m_visionBack, m_visionLeft, m_visionRight);

    m_testArm = new TestArmSubsystem();

    // Configure the button bindings
    configureButtonBindings();

    // Set initial default command for teleop
    setTeleopModeDefaultCommand();

    if (m_driverController.getStartButton()) {
        m_robotDrive.zeroGyro();
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, Button.kStart.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.zeroGyro(),
            m_robotDrive)); 
            
    new JoystickButton(m_driverController, Button.kA.value)
        .whileTrue(new RunCommand(
            () -> m_testArm.drive(50),
            m_testArm))
        .whileFalse(new RunCommand(
            () -> m_testArm.stopMotor(),
            m_testArm));   
                
    new JoystickButton(m_driverController, Button.kB.value)
        .whileTrue(new RunCommand(
            () -> m_testArm.drive(-50),
            m_testArm))
        .whileFalse(new RunCommand(
            () -> m_testArm.stopMotor(),
            m_testArm));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // NOTE: All poses use blue origin coordinates (field origin at blue alliance corner).
    // PathPlanner automatically mirrors paths for red alliance.
    // The gyro will be zeroed in autonomousInit() using zeroGyroWithAlliance().
    //
    // If you need to reset the pose to a specific location:
    // m_robotDrive.resetOdometry(new Pose2d(x, y, rotation));

    return new RunCommand(() -> {
    });
  }

  /**
   * Configures the default command for teleop mode.
   * Uses YAGSL's SwerveInputStream with alliance-relative control for automatic
   * coordinate flipping on red alliance.
   *
   * Alliance-relative control ensures that "forward" on the joystick always means
   * "away from our alliance wall" regardless of which alliance we're on.
   */
  public void setTeleopModeDefaultCommand() {
    // Create alliance-aware field-relative drive input stream
    // SwerveInputStream handles alliance-based coordinate flipping automatically
    SwerveInputStream driveInputStream = SwerveInputStream.of(
        m_robotDrive.getSwerveDrive(),
        () -> -m_driverController.getLeftY(),  // Forward/backward (inverted for controller convention)
        () -> -m_driverController.getLeftX())  // Left/right (inverted for controller convention)
      .withControllerRotationAxis(() -> -m_driverController.getRightX())  // Rotation
      .deadband(OIConstants.kDriveDeadband)
      .allianceRelativeControl(true);  // Enable automatic alliance-based coordinate flipping

    // Convert to command and set as default
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.getSwerveDrive().drive(driveInputStream.get()),
            m_robotDrive
        )
    );
  }

  /**
   * Configures the default command for test mode.
   * Return-to-home behavior with automatic pathfinding.
   */
  public void setTestModeDefaultCommand() {
    m_robotDrive.setDefaultCommand(
        m_robotDrive.returnToHomeCommand(
            () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
            () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
            () -> -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband)
        )
    );
  }

  /**
   * Zeros the gyroscope. Should be called when robot is facing red alliance wall.
   * Required for proper odometry initialization in autonomous mode.
   */
  public void zeroGyro() {
    m_robotDrive.zeroGyro();
  }
}