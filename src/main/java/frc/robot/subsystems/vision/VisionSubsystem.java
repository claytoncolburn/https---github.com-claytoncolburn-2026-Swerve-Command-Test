package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.BotPose;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;

/**
 * VisionSubsystem provides a lightweight wrapper around YALL (Yet Another Limelight Library)
 * for AprilTag-based robot localization using Limelight's MegaTag2 algorithm.
 *
 * This subsystem is designed to be called from YAGSLDriveSubsystem's background vision thread,
 * not to run autonomously in periodic(). It provides pose estimates and calculates appropriate
 * standard deviations based on tag count and distance for robust vision fusion.
 */
public class VisionSubsystem extends SubsystemBase {
    private final Limelight limelight;
    private final LimelightPoseEstimator estimator;

    // Vision confidence thresholds
    private static final double MULTI_TAG_CLOSE_STD_DEV = 0.01;  // meters (very confident)
    private static final double SINGLE_TAG_BASE_STD_DEV = 0.5;   // meters (less confident)
    private static final double MAX_DISTANCE_FOR_VISION = 6.0;   // meters (reject beyond this)

    /**
     * Creates a new VisionSubsystem.
     *
     * @param limelightName The network table name of the Limelight (typically "limelight")
     */
    public VisionSubsystem(String limelightName) {
        this.limelight = new Limelight(limelightName);
        this.estimator = this.limelight.getPoseEstimator(true);

        // Configure Limelight to use AprilTag pipeline (pipeline 0)
        limelight.getSettings()
            .withPipelineIndex(0)
            .save();
    }

    /**
     * Updates the Limelight with current robot orientation data.
     * This is required for accurate MegaTag2 pose estimation.
     * Should be called from the vision loop before getting pose estimates.
     *
     * @param rotation Current robot rotation from gyro (Rotation3d)
     * @param yawRate Current angular velocity in degrees per second
     */
    public Optional<PoseEstimate> updateRobotOrientation(Rotation3d rotation, double yawRate) {
        // Create Orientation3d with rotation and angular velocity
        // Note: YALL expects angular velocity as a vector [0, 0, yawRate]
        Orientation3d orientation = new Orientation3d(
            rotation,new AngularVelocity3d(
                        DegreesPerSecond.of(0), // Pitch rate (assume 0 for flat field)
                        DegreesPerSecond.of(0), // Roll rate (assume 0 for flat field)
                        DegreesPerSecond.of(yawRate) // Yaw rate
                    )
        );

        // Send orientation to Limelight for MegaTag2 processing
        limelight.getSettings()
            .withRobotOrientation(orientation)
            .save();

        return this.getPoseEstimate();
    }

    /**
     * Gets the current robot pose estimate from Limelight MegaTag2.
     *
     * @return Optional containing PoseEstimate if valid tags are visible, empty otherwise
     */
    public Optional<PoseEstimate> getPoseEstimate() {
        // Use Blue alliance MegaTag2 by default
        // Note: This should be swapped based on alliance color in a real implementation
        return estimator.getAlliancePoseEstimate();
    }

    /**
     * Calculates standard deviations for vision measurement based on tag count and distance.
     *
     * Standard deviation strategy:
     * - Multiple tags + close distance: Very low XY stddev (high confidence)
     * - Single tag: Scales with distance squared (less confident, worse at distance)
     * - Rotation stddev: Always infinite (gyro is authoritative for heading)
     *
     * @param estimate The pose estimate to calculate standard deviations for
     * @return Matrix<N3, N1> containing [x_stddev, y_stddev, theta_stddev]
     */
    public Matrix<N3, N1> getStandardDeviations(PoseEstimate estimate) {
        // Reject measurements that are too far away or invalid
        if (estimate.avgTagDist > MAX_DISTANCE_FOR_VISION) {
            // Return very high standard deviations (essentially reject this measurement)
            return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }

        double xyStdDev;

        if (estimate.tagCount >= 2) {
            // Multiple tags: high confidence, use low standard deviation
            xyStdDev = MULTI_TAG_CLOSE_STD_DEV;
        } else {
            // Single tag: confidence decreases with distance squared
            // This penalizes distant single-tag measurements appropriately
            xyStdDev = SINGLE_TAG_BASE_STD_DEV * Math.pow(estimate.avgTagDist, 2);
        }

        // Always use infinite rotation standard deviation - gyro is authoritative
        // This prevents vision from fighting the gyro on heading
        return VecBuilder.fill(xyStdDev, xyStdDev, Double.MAX_VALUE);
    }

    /**
     * Gets the underlying Limelight instance for advanced configuration.
     *
     * @return The Limelight instance
     */
    public Limelight getLimelight() {
        return limelight;
    }

    @Override
    public void periodic() {
        // Intentionally empty - this subsystem is called from YAGSLDriveSubsystem's
        // background thread, not from periodic()
    }
}
