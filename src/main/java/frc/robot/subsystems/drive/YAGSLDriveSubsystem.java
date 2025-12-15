package frc.robot.subsystems.drive;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionSubsystem;
import limelight.networktables.PoseEstimate;
import swervelib.SwerveDrive;
import swervelib.imu.SwerveIMU;
import swervelib.parser.SwerveParser;

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

            // Initialize gyro to heading 0 (facing X+ for Blue, X- for Red alliance)
            // This ensures the limelights receive correct orientation data from the first query
            swerveDrive.zeroGyro();

            // Simulation-specific configuration
            if (RobotBase.isSimulation()) {
                // Disable features that don't work well in simulation
                // Heading correction: Continuously adjusts heading based on setpoint
                // Cosine compensation: Adjusts for heading error in field-relative mode
                swerveDrive.setHeadingCorrection(false);
                swerveDrive.setCosineCompensator(false);
            }

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
     * Zeros the gyroscope, setting the current heading as forward (0 degrees).
     *
     * IMPORTANT: In YAGSL 2025, this must be called explicitly in autonomous init.
     * It is no longer called automatically.
     */
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    /**
     * Locks the swerve modules in an X pattern to resist being pushed.
     * Useful for defensive positioning or maintaining position on ramps.
     */
    public void setX() {
        swerveDrive.lockPose();
    }

    /**
     * Gets the current chassis speeds.
     *
     * @return Current velocity of the robot
     */

    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

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

        // Pose telemetry
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

    @Override
    public void close() throws Exception {
        // Don't forget to clean up your mess!
        this.swerveDrive.close();
        this.visionNotifier.close();
    }
}
