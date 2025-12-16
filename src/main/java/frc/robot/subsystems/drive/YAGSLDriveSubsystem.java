package frc.robot.subsystems.drive;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionSubsystem;
import limelight.networktables.PoseEstimate;
import swervelib.SwerveDrive;
import swervelib.imu.SwerveIMU;
import swervelib.parser.SwerveParser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;

import java.io.File;
import java.io.IOException;
import java.util.Optional;

/**
 * YAGSLDriveSubsystem manages the swerve drive using YAGSL (Yet Another Generic Swerve Library).
 *
 * This subsystem loads configuration from JSON files and provides:
 * - Basic swerve driving functionality
 * - Pose estimation using wheel odometry
 * - Vision fusion for improved localization
 * - Multiple Limelight support for redundant vision measurements
 *
 * Key design decisions:
 * - Uses YAGSL's built-in SwerveDrivePoseEstimator for sensor fusion
 * - Runs vision processing on background thread via Notifier (100Hz)
 * - Supports multiple VisionSubsystems (limelights) for redundancy
 * - Initializes with heading 0 (facing X+ for Blue, X- for Red alliance)
 * - Disables heading correction and cosine compensation in simulation for realism
 */
public class YAGSLDriveSubsystem extends SubsystemBase implements AutoCloseable {
    private final SwerveDrive swerveDrive;
    private final VisionSubsystem[] visionSubsystems;
    private final Notifier visionNotifier;
    @Logged private final Field2d field;

    // Vision telemetry (updated from vision loop, read from periodic)
    // Remind me to tell you a story about `volatile` sometime... -Sean
    @Logged private volatile int lastTagCount = 0;
    @Logged private volatile double lastAvgTagDist = 0.0;
    @Logged private volatile boolean lastVisionValid = false;

    // Maximum speeds (from YAGSL config)
    private static final double MAX_SPEED = 4.8; // m/s

    // Vision update rate (100Hz for low latency)
    private static final double VISION_UPDATE_PERIOD_SECONDS = 0.01;

    /**
     * Creates a new YAGSLDriveSubsystem by loading configuration from JSON files.
     *
     * The configuration directory should be at src/main/deploy/swerve/ and contain:
     * - swervedrive.json (main config)
     * - physicalproperties.json (conversion factors, gear ratios)
     * - controllerproperties.json (PID values)
     * - modules/frontleft.json, frontright.json, backleft.json, backright.json
     *
     * @param visionSubsystems One or more vision subsystems for AprilTag-based localization
     */
    public YAGSLDriveSubsystem(VisionSubsystem... visionSubsystems) {
        this.visionSubsystems = visionSubsystems;

        try {
            // Get the deploy directory where YAGSL JSON configs are stored
            File swerveDirectory = new File(Filesystem.getDeployDirectory(), "swerve");

            // Parse JSON configuration and create SwerveDrive instance
            // Maximum speed is set in swervedrive.json
            this.swerveDrive = new SwerveParser(swerveDirectory)
                .createSwerveDrive(MAX_SPEED);

            // Initialize gyro to heading 0 temporarily for startup
            // Proper alliance-aware zeroing will occur in autonomousInit() via zeroGyro()
            // which sets heading to 0° for blue alliance or 180° for red alliance
            swerveDrive.zeroGyro();

            // Simulation-specific configuration
            if (RobotBase.isSimulation()) {
                // Disable features that don't work well in simulation
                // Heading correction: Continuously adjusts heading based on setpoint
                // Cosine compensation: Adjusts for heading error in field-relative mode
                swerveDrive.setHeadingCorrection(false);
                swerveDrive.setCosineCompensator(false);
            }

            // Configure PathPlanner AutoBuilder for pathfinding and auto
            configurePathPlanner();

        } catch (IOException e) {
            throw new RuntimeException("Failed to load YAGSL swerve configuration", e);
        }

        // Initialize Field2d for visualization
        this.field = new Field2d();
        SmartDashboard.putData("Field", field);

        // Start background vision thread at 100Hz
        // This runs asynchronously to avoid blocking the main thread with NetworkTables I/O
        this.visionNotifier = new Notifier(this::runVisionLoop);
        this.visionNotifier.startPeriodic(VISION_UPDATE_PERIOD_SECONDS);
    }

    /**
     * Drives the robot using field-relative or robot-relative control.
     *
     * @param translation   Translation vector (x: forward+, y: left+) in m/s
     * @param rotation      Rotation rate in rad/s (CCW+)
     * @param fieldRelative If true, translation is relative to field. If false, relative to robot.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation, rotation, fieldRelative, false);
    }

    /**
     * Drives the robot using individual axis velocities.
     *
     * @param vxMetersPerSecond Forward velocity (m/s)
     * @param vyMetersPerSecond Sideways velocity (m/s, left positive)
     * @param omegaRadiansPerSecond Rotation rate (rad/s, CCW positive)
     * @param fieldRelative If true, velocities are relative to field. If false, relative to robot.
     */
    public void drive(double vxMetersPerSecond, double vyMetersPerSecond,
                     double omegaRadiansPerSecond, boolean fieldRelative) {
        swerveDrive.drive(
            new Translation2d(vxMetersPerSecond, vyMetersPerSecond),
            omegaRadiansPerSecond,
            fieldRelative,
            false  // isOpenLoop
        );
    }

    /**
     * Drives the robot using ChassisSpeeds.
     *
     * @param chassisSpeeds Desired chassis speeds
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        swerveDrive.drive(chassisSpeeds);
    }

    /**
     * Gets the current robot pose from the pose estimator.
     *
     * @return The current estimated pose (includes wheel odometry and vision fusion)
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Resets the robot's pose to a specific position.
     *
     * @param pose The pose to reset to
     */
    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    /**
     * Zeros the gyroscope with alliance-aware heading initialization.
     *
     * Blue origin convention requires:
     * - Blue alliance (facing away from blue DS): heading = 0° (facing X+, toward red wall)
     * - Red alliance (facing away from red DS): heading = 180° (facing X-, toward blue wall)
     *
     * The robot is always positioned facing away from the driver station at match start.
     * This method accounts for alliance color to set the correct heading in blue origin coordinates.
     *
     * In YAGSL 2025.7.1+, gyro zeroing is no longer automatic at startup.
     * Call this method in autonomousInit() to ensure proper odometry initialization.
     */
    public void zeroGyro() {
        // Get current alliance color
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            // Red alliance: Robot faces blue wall (X-), so heading should be 180°
            swerveDrive.zeroGyro();
            // Reset odometry to add 180° offset while keeping current position
            Pose2d currentPose = swerveDrive.getPose();
            swerveDrive.resetOdometry(new Pose2d(
                currentPose.getTranslation(),
                currentPose.getRotation().plus(new Rotation2d(Math.PI))  // Add 180°
            ));
            SmartDashboard.putString("Drive/Gyro Alliance", "Red (180°)");
        } else {
            // Blue alliance (or unknown): Robot faces red wall (X+), so heading is 0°
            swerveDrive.zeroGyro();
            SmartDashboard.putString("Drive/Gyro Alliance", "Blue (0°)");
        }

        SmartDashboard.putBoolean("Drive/Gyro Zeroed", true);
    }

    /**
     * Locks the swerve modules in an X pattern to resist being pushed.
     * Useful for defensive positioning or maintaining position on ramps.
     */
    public void setX() {
        swerveDrive.lockPose();
    }

    /**
     * Gets the current chassis speeds in field-relative units.
     *
     * @return Current velocity of the robot
     */
    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    /**
     * Gets the current chassis speeds in robot-relative units.
     *
     * @return Current velocity of the robot
     */
    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Gets the underlying YAGSL SwerveDrive instance for advanced usage.
     *
     * @return The SwerveDrive instance
     */
    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    /**
     * Vision processing loop that runs asynchronously at 100Hz.
     *
     * This method:
     * 1. Gets current gyro orientation and angular velocity
     * 2. Updates all VisionSubsystems with orientation data (required for MegaTag2)
     * 3. Retrieves pose estimates from each VisionSubsystem
     * 4. Adds vision measurements to pose estimator with appropriate standard deviations
     *
     * Runs on background thread via Notifier to avoid blocking main thread with NetworkTables I/O.
     * Processes all vision subsystems to enable multi-camera setups for redundancy and wider coverage.
     */
    private void runVisionLoop() {
        try {
            // Get gyro data from YAGSL's IMU
            SwerveIMU imu = swerveDrive.getGyro();
            Rotation3d rotation = imu.getRotation3d();
            double yawRate = imu.getYawAngularVelocity().in(Units.DegreesPerSecond);

            // Track aggregated telemetry across all vision subsystems
            int totalTagCount = 0;
            double totalAvgTagDist = 0.0;
            int validEstimates = 0;

            // Process each vision subsystem
            for (VisionSubsystem visionSubsystem : visionSubsystems) {
                // Update vision subsystem with current orientation
                // And retrieve back the vision system's integrated pose estimate
                Optional<PoseEstimate> estimate = visionSubsystem.updateRobotOrientation(rotation, yawRate);

                if (estimate.isPresent()) {
                    PoseEstimate est = estimate.get();

                    // Add vision measurements to pose estimator
                    if (est.tagCount > 0) {
                        // Add vision measurement with dynamic standard deviations
                        // Standard deviations are calculated based on tag count and distance
                        swerveDrive.addVisionMeasurement(
                            est.pose.toPose2d(),
                            est.timestampSeconds,
                            visionSubsystem.getStandardDeviations(est)
                        );

                        // Aggregate telemetry
                        totalTagCount += est.tagCount;
                        totalAvgTagDist += est.avgTagDist;
                        validEstimates++;
                    }
                }
            }

            // Update telemetry data (thread-safe using volatile)
            // Show aggregated data across all vision subsystems
            lastTagCount = totalTagCount;
            lastAvgTagDist = validEstimates > 0 ? totalAvgTagDist / validEstimates : 0.0;
            lastVisionValid = validEstimates > 0;

        } catch (Exception e) {
            // Catch any exceptions to prevent the Notifier from stopping
            // Log the error but don't crash the vision loop
            System.err.println("Error in vision loop: " + e.getMessage());
            e.printStackTrace();
        }
    }

    @Override
    public void periodic() {
        // YAGSL handles module updates internally
        // Vision fusion runs on background thread via visionNotifier

        // Update telemetry
        Pose2d currentPose = getPose();

        // Alliance color telemetry
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            SmartDashboard.putString("Drive/Alliance", alliance.get().toString());
            SmartDashboard.putBoolean("Drive/Is Red Alliance", alliance.get() == DriverStation.Alliance.Red);
        } else {
            SmartDashboard.putString("Drive/Alliance", "UNKNOWN");
            SmartDashboard.putBoolean("Drive/Is Red Alliance", false);
        }

        // Pose telemetry (always in blue origin coordinates)
        SmartDashboard.putNumber("Drive/Pose X", currentPose.getX());
        SmartDashboard.putNumber("Drive/Pose Y", currentPose.getY());
        SmartDashboard.putNumber("Drive/Pose Rotation", currentPose.getRotation().getDegrees());

        // Field2d visualization
        field.setRobotPose(currentPose);

        // Vision telemetry (from background thread)
        SmartDashboard.putNumber("Vision/Tag Count", lastTagCount);
        SmartDashboard.putNumber("Vision/Avg Tag Distance", lastAvgTagDist);
        SmartDashboard.putBoolean("Vision/Valid", lastVisionValid);

        // Velocity telemetry
        ChassisSpeeds velocity = getFieldVelocity();
        SmartDashboard.putNumber("Drive/Velocity X", velocity.vxMetersPerSecond);
        SmartDashboard.putNumber("Drive/Velocity Y", velocity.vyMetersPerSecond);
        SmartDashboard.putNumber("Drive/Velocity Omega", Math.toDegrees(velocity.omegaRadiansPerSecond));
    }

    @Override
    public void simulationPeriodic() {
        // YAGSL handles simulation updates internally via MapleSim integration
    }

    /**
     * Creates a command for test mode that records a "home" pose and autonomously
     * returns to it when the driver releases the joystick.
     *
     * Behavior:
     * - IDLE: Joystick in deadband, no home pose recorded → do nothing
     * - MANUAL_DRIVE: Joystick active → record home pose (once), then drive manually
     * - PATHFINDING: Joystick in deadband, home pose exists → pathfind back to home
     *
     * @param xSupplier Forward/backward joystick input (-1.0 to 1.0)
     * @param ySupplier Left/right joystick input (-1.0 to 1.0)
     * @param rotSupplier Rotation joystick input (-1.0 to 1.0)
     * @return Command that alternates between manual drive and return-to-home pathfinding
     */
    public Command returnToHomeCommand(DoubleSupplier xSupplier,
                                        DoubleSupplier ySupplier,
                                        DoubleSupplier rotSupplier) {
        // State variable: home pose (captured in closure, persists across command executions)
        AtomicReference<Pose2d> homePose = new AtomicReference<>(null);

        return Commands.either(
            // Branch 1: Joystick in deadband → pathfind to home (if home exists)
            Commands.defer(() -> {
                if (homePose.get() != null) {
                    SmartDashboard.putString("Test/Mode", "PATHFINDING");
                    return pathfindToPose(homePose.get(), getReturnVelocity());
                } else {
                    SmartDashboard.putString("Test/Mode", "IDLE");
                    return Commands.none();
                }
            }, Set.of(this)),

            // Branch 2: Joystick active → manual drive + record home
            Commands.run(() -> {
                // Record home on first movement (if not already set)
                if (homePose.get() == null) {
                    homePose.set(getPose());
                    SmartDashboard.putBoolean("Test/Home Set", true);
                    SmartDashboard.putNumber("Test/Home X", homePose.get().getX());
                    SmartDashboard.putNumber("Test/Home Y", homePose.get().getY());
                    SmartDashboard.putNumber("Test/Home Rotation", homePose.get().getRotation().getDegrees());
                }

                SmartDashboard.putString("Test/Mode", "MANUAL");

                // Manual field-relative drive
                drive(xSupplier.getAsDouble(),
                      ySupplier.getAsDouble(),
                      rotSupplier.getAsDouble(),
                      true); // field-relative
            }, this),

            // Condition: Are all sticks in deadband?
            () -> isInDeadband(xSupplier.getAsDouble(), ySupplier.getAsDouble(), rotSupplier.getAsDouble())
        );
    }

    /**
     * Checks if all joystick inputs are within deadband.
     *
     * @param x Forward/backward input
     * @param y Left/right input
     * @param rot Rotation input
     * @return true if all inputs are within deadband (|input| < 0.1)
     */
    private boolean isInDeadband(double x, double y, double rot) {
        return Math.abs(x) < 0.1 && Math.abs(y) < 0.1 && Math.abs(rot) < 0.1;
    }

    /**
     * Gets the return velocity from SmartDashboard preferences.
     *
     * @return Velocity in m/s for return-to-home pathfinding
     */
    private double getReturnVelocity() {
        return Preferences.getDouble("Test/Return Velocity", 1.0);
    }

    /**
     * Creates a command that pathfinds to a target pose.
     *
     * @param targetPose The pose to pathfind to
     * @param maxVelocity Maximum velocity during pathfinding (m/s)
     * @return Command that pathfinds to the target pose
     */
    private Command pathfindToPose(Pose2d targetPose, double maxVelocity) {
        PathConstraints constraints = new PathConstraints(
            maxVelocity,
            3.0,  // Conservative acceleration for safety
            Math.toRadians(540),  // 540 degrees/sec max angular velocity
            Math.toRadians(720)   // 720 degrees/sec max angular acceleration
        );

        return AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0  // Goal end velocity (stop at target)
            )
            .finallyDo((interrupted) -> {
                stopDrive();
                SmartDashboard.putString("Test/Mode", interrupted ? "INTERRUPTED" : "AT HOME");
            });
    }

    /**
     * Stops all drive motors by commanding zero velocity.
     */
    private void stopDrive() {
        drive(new ChassisSpeeds(0, 0, 0));
    }

    /**
     * Configures PathPlanner's AutoBuilder for autonomous pathfinding and path following.
     * Sets up tunable PID controllers and loads robot configuration.
     */
    private void configurePathPlanner() {
        try {
            // Initialize PID preferences with defaults (only sets if not already present)
            Preferences.initDouble("PathPlanner/Translation P", 5.0);
            Preferences.initDouble("PathPlanner/Translation I", 0.0);
            Preferences.initDouble("PathPlanner/Translation D", 0.0);
            Preferences.initDouble("PathPlanner/Rotation P", 4.0);
            Preferences.initDouble("PathPlanner/Rotation I", 0.0);
            Preferences.initDouble("PathPlanner/Rotation D", 0.0);
            Preferences.initDouble("Test/Return Velocity", 1.0);

            // Load PID values from preferences
            PIDConstants translationPID = new PIDConstants(
                Preferences.getDouble("PathPlanner/Translation P", 5.0),
                Preferences.getDouble("PathPlanner/Translation I", 0.0),
                Preferences.getDouble("PathPlanner/Translation D", 0.0)
            );

            PIDConstants rotationPID = new PIDConstants(
                Preferences.getDouble("PathPlanner/Rotation P", 4.0),
                Preferences.getDouble("PathPlanner/Rotation I", 0.0),
                Preferences.getDouble("PathPlanner/Rotation D", 0.0)
            );

            // Load robot configuration from deploy directory
            RobotConfig config = RobotConfig.fromGUISettings();

            // Configure AutoBuilder with holonomic drive controller
            AutoBuilder.configure(
                this::getPose,                    // Supplier<Pose2d>: Robot pose
                this::resetOdometry,              // Consumer<Pose2d>: Reset pose
                this::getRobotVelocity,           // Supplier<ChassisSpeeds>: Current robot-relative speeds
                this::drive,                      // Consumer<ChassisSpeeds>: Drive with chassis speeds
                new PPHolonomicDriveController(translationPID, rotationPID),
                config,
                () -> {
                    // Mirror paths for red alliance (blue origin convention)
                    // PathPlanner will automatically flip X coordinates and rotations
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                },
                this
            );

            // Enable PathPlanner warmup to avoid JIT delays on first use
            PathfindingCommand.warmupCommand().schedule();

        } catch (Exception e) {
            System.err.println("Failed to configure PathPlanner: " + e.getMessage());
            e.printStackTrace();
        }
    }

    @Override
    public void close() throws Exception {
        // Don't forget to clean up your mess!
        this.swerveDrive.close();
        this.visionNotifier.close();
    }
}
