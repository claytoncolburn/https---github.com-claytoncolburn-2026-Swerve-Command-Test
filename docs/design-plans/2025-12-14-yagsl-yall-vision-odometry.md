# YAGSL + YALL Vision-Enabled Odometry Design

## Overview

Implement vision-enabled odometry for FRC 2025 using YAGSL (Yet Another Generic Swerve Library) and YALL (Yet Another Limelight Library) to achieve accurate autonomous positioning with zero-configuration auto-localization.

**Goals:**
- Accurate autonomous positioning through vision-corrected odometry
- Persistent pose estimation across disabled/teleop/autonomous modes
- Zero-config auto-localization when robot detects AprilTags
- Dashboard visualization of pose, vision confidence, and odometry data
- Full simulation support for testing before robot deployment

**Success Criteria:**
- Robot auto-localizes position upon seeing 2+ AprilTags without manual setup
- Vision fusion corrects wheel odometry drift during autonomous routines
- Pose remains accurate throughout match without requiring re-initialization
- All functionality testable in simulation using MapleSim

## Architecture

**Approach:** "Drive pulls from Vision" with asynchronous vision processing.

Two new subsystems in `src/main/java/frc/robot/subsystems/`:

**VisionSubsystem** - Lightweight wrapper around YALL's Limelight API:
- Initializes Limelight connection with 2025 Reefscape field configuration
- Provides `updateRobotOrientation(Rotation3d, double yawRate)` - updates Limelight with robot orientation (required for MegaTag2 solver)
- Provides `getPoseEstimate()` - returns `Optional<PoseEstimate>` from MegaTag2 with pose, timestamp, tag count, and distance
- Provides `getStandardDeviations(PoseEstimate)` - calculates dynamic vision trust based on tag count and distance
- No periodic processing - all methods called from YAGSLDriveSubsystem's background thread

**YAGSLDriveSubsystem** - Manages swerve drive and orchestrates vision fusion:
- Loads YAGSL configuration from JSON files in `src/main/deploy/swerve/`
- Takes `VisionSubsystem` as constructor dependency (dependency injection)
- Runs Notifier at 100Hz for asynchronous vision processing (`runVisionLoop()`)
- Background thread: gets gyro data, updates vision orientation, pulls pose estimates, calls `swerveDrive.addVisionMeasurement()`
- Main thread periodic(): processes drive commands, updates telemetry
- Provides drive control: `drive()`, `getPose()`, `resetOdometry()`, `zeroGyro()`
- Implements auto-localization on first valid vision measurement (2+ tags, <4m distance)

**Data Flow:**
1. Main thread (20ms): YAGSLDriveSubsystem.periodic() processes drive commands
2. Background thread (10ms): runVisionLoop() gets gyro → updates VisionSubsystem → pulls pose estimate → adds to YAGSL pose estimator
3. NetworkTables I/O (blocking operations) happens in background thread, preventing main thread delays
4. YAGSL's `addVisionMeasurement()` is thread-safe with internal locking

**Configuration:**
- JSON files in `src/main/deploy/swerve/` define motor controllers (SparkFlex drive, SparkMax turn), PIDs, gear ratios, and physical properties
- Migrates existing `Constants.java` values (CAN IDs, PIDs, wheelbase, gear ratios) to YAGSL JSON format

## Existing Patterns

**Codebase investigation found:**
- Complete swerve implementation in `src/main/java/frc/robot/subsystems/drive/` using WPILib's standard approach
- DriveSubsystem.java (211 lines): NavX gyro, SwerveDriveOdometry, four SwerveModule instances
- SwerveModule.java (151 lines): REV SparkFlex/SparkMax control with analog absolute encoders, closed-loop PID
- Pattern: Subsystems extend SubsystemBase, use dependency injection in constructor, manage periodic updates
- YAGSL and YALL vendordeps already installed (yagsl.json v2025.8.0, yall.json v2025.9.6) but not yet integrated

**This design follows existing patterns:**
- Subsystems extend SubsystemBase (matches DriveSubsystem pattern)
- Dependency injection in constructors (VisionSubsystem injected into YAGSLDriveSubsystem)
- Separate subsystems for distinct concerns (matches DriveSubsystem + TestArmSubsystem structure)
- REV hardware control via vendor libraries (matches existing SparkFlex/SparkMax usage)

**This design diverges:**
- Uses JSON configuration instead of Constants.java hardcoded values (YAGSL design philosophy)
- Introduces Notifier for asynchronous processing (existing code is purely synchronous)
- Uses YAGSL's SwerveDrivePoseEstimator instead of manual WPILib SwerveDriveOdometry (built-in vision fusion)

**Justification for divergence:**
- JSON configuration enables faster tuning without code recompilation
- Asynchronous vision processing prevents NetworkTables I/O from blocking main robot loop (critical for consistent cycle timing)
- YAGSL's built-in pose estimator simplifies vision integration compared to manual implementation

**Coexistence:**
- Existing DriveSubsystem in `src/main/java/frc/robot/subsystems/drive/` remains unchanged for backwards compatibility
- New YAGSL implementation lives alongside but not instantiated in RobotContainer yet (exclusive hardware control - cannot run both simultaneously)
- When ready to test, comment out old DriveSubsystem and uncomment YAGSLDriveSubsystem in RobotContainer.java

## Implementation Phases

### Phase 1: YAGSL Configuration Files
**Goal:** Create JSON configuration files that define swerve hardware setup for YAGSL

**Components:**
- Create: `src/main/deploy/swerve/swervedrive.json` - main config (gyro, max speeds, module positions)
- Create: `src/main/deploy/swerve/controllerproperties.json` - PID values (drive P=0.04, angle P=0.03)
- Create: `src/main/deploy/swerve/physicalproperties.json` - conversion factors (6.75:1 gear ratio, 0.1016m wheel diameter)
- Create: `src/main/deploy/swerve/modules/frontleft.json` - front-left module (drive CAN 1, angle CAN 2, encoder port 0)
- Create: `src/main/deploy/swerve/modules/frontright.json` - front-right module (drive CAN 7, angle CAN 8, encoder port 3)
- Create: `src/main/deploy/swerve/modules/backleft.json` - back-left module (drive CAN 3, angle CAN 4, encoder port 1)
- Create: `src/main/deploy/swerve/modules/backright.json` - back-right module (drive CAN 5, angle CAN 6, encoder port 2)

**Dependencies:** None (first phase)

**Testing:**
- Deploy to robot and verify JSON files parse without errors
- Check YAGSL documentation examples match configuration structure
- Validate CAN IDs match existing Constants.java values (lines 87-94)

**Migration from Constants.java:**
- Wheelbase: 0.5207m (Constants.java:167)
- Max speed: 4.8 m/s (Constants.java:165)
- Drive motor: SparkFlex NEO on CAN IDs 1,3,5,7
- Turn motor: SparkMax NEO on CAN IDs 2,4,6,8
- Absolute encoders: Analog ports 0-3 with chassis angular offsets from Constants.java:156-159

### Phase 2: VisionSubsystem Implementation
**Goal:** Create VisionSubsystem that wraps YALL and provides pose estimates to drive subsystem

**Components:**
- Create: `src/main/java/frc/robot/subsystems/VisionSubsystem.java`
- Modify: `build.gradle` if YALL dependencies need explicit declaration (verify vendordeps sufficient)

**Dependencies:** Phase 1 complete (need project structure in place)

**Testing:**
- Unit test: VisionSubsystem initializes Limelight without errors
- Integration test: Can retrieve pose estimate in simulation (even if returning empty Optional)
- Verify NetworkTables connection to Limelight works

**Key Methods:**
```java
public VisionSubsystem(String limelightName)
public void updateRobotOrientation(Rotation3d rotation, double yawRate)
public Optional<PoseEstimate> getPoseEstimate()
public Matrix<N3, N1> getStandardDeviations(PoseEstimate estimate)
```

**Implementation Details:**
- Initialize: `limelight = new Limelight("limelight")`, set pipeline to AprilTag detection
- `updateRobotOrientation()`: Calls `limelight.getSettings().withRobotOrientation(...).save()`
- `getPoseEstimate()`: Returns `BotPose.BLUE_MEGATAG2.get(limelight)`
- `getStandardDeviations()`: Multi-tag + close (<3m) = [0.01, 0.01, 999999], single tag scales with distance squared, infinite rotation stddev (gyro is truth)

### Phase 3: YAGSLDriveSubsystem Core Implementation
**Goal:** Implement basic YAGSLDriveSubsystem with YAGSL integration, no vision yet

**Components:**
- Create: `src/main/java/frc/robot/subsystems/YAGSLDriveSubsystem.java`

**Dependencies:** Phase 1 complete (JSON configs exist), Phase 2 complete (VisionSubsystem exists for dependency injection)

**Testing:**
- Robot can drive using YAGSL (field-relative and robot-relative)
- `getPose()` returns odometry-only pose (no vision yet)
- `zeroGyro()` and `resetOdometry()` work correctly
- Module states update correctly on dashboard

**Key Methods:**
```java
public YAGSLDriveSubsystem(VisionSubsystem visionSubsystem)
public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative)
public Pose2d getPose()
public void resetOdometry(Pose2d pose)
public void zeroGyro()
public void periodic()
```

**Implementation Details:**
- Load config: `swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive(...)`
- Store VisionSubsystem reference (not used yet - Phase 4 will integrate)
- Implement drive control using `swerveDrive.drive()`
- Call `swerveDrive.zeroGyro()` explicitly in autonomous init (2025 YAGSL breaking change - no auto-zeroing)

### Phase 4: Vision Fusion with Notifier
**Goal:** Add asynchronous vision processing to integrate vision measurements into pose estimation

**Components:**
- Modify: `src/main/java/frc/robot/subsystems/YAGSLDriveSubsystem.java` - add Notifier and runVisionLoop()

**Dependencies:** Phase 3 complete (YAGSLDriveSubsystem drives robot), Phase 2 complete (VisionSubsystem provides estimates)

**Testing:**
- Vision measurements appear in pose estimator (verify via telemetry)
- Notifier runs at 100Hz (check timing in dashboard)
- Pose estimate corrects when robot sees AprilTags in simulation
- No delays or stuttering in main robot loop (async processing working)

**Implementation Details:**
- Add field: `private final Notifier visionNotifier`
- Create method: `private void runVisionLoop()`
- In constructor: `visionNotifier = new Notifier(this::runVisionLoop); visionNotifier.startPeriodic(0.01);`
- In runVisionLoop():
  1. Get gyro: `swerveIMU.getRotation3d()` and `getYawAngularVelocity()`
  2. Update vision: `visionSubsystem.updateRobotOrientation(rotation, yawRate)`
  3. Get estimate: `Optional<PoseEstimate> est = visionSubsystem.getPoseEstimate()`
  4. If valid: `swerveDrive.addVisionMeasurement(est.pose, est.timestampSeconds, visionSubsystem.getStandardDeviations(est))`

**Standard Deviations Tuning:**
- Start conservative: Multi-tag [0.1, 0.1, 999999], single tag [0.5, 0.5, 999999]
- Tune based on observed accuracy in testing
- Rotation stddev infinite (999999) because gyro is authoritative for MegaTag2

### Phase 5: Auto-Localization Implementation
**Goal:** Implement zero-config auto-localization that sets initial pose when robot first sees AprilTags

**Components:**
- Modify: `src/main/java/frc/robot/subsystems/YAGSLDriveSubsystem.java` - add auto-localization logic

**Dependencies:** Phase 4 complete (vision fusion working)

**Testing:**
- Robot in simulation: starts at (0,0), drives near AprilTag, auto-localizes to correct position
- Robot on field: enable robot, confirm auto-localization occurs within 1-2 seconds of seeing tags
- Dashboard shows "HasAutoLocalized" = true after initialization
- Manual `resetOdometry()` call still works (override auto-localization)

**Implementation Details:**
- Add field: `private boolean hasAutoLocalized = false`
- In runVisionLoop(), before adding vision measurement:
  ```java
  if (!hasAutoLocalized && est.isPresent()) {
      PoseEstimate pose = est.get();
      if (pose.isValid && pose.tagCount >= 2 && pose.avgTagDist < 4.0) {
          swerveDrive.resetOdometry(pose.pose);
          hasAutoLocalized = true;
          // Publish to NetworkTables: SmartDashboard/Drive/HasAutoLocalized
      }
  }
  ```
- Add method: `public void forceResetOdometry(Pose2d pose)` for manual override if needed
- Publish `hasAutoLocalized` flag to telemetry

**Auto-Localization Criteria:**
- 2+ AprilTags visible (high confidence)
- Average distance < 4 meters (reasonable accuracy)
- Valid MegaTag2 pose estimate

### Phase 6: Telemetry Integration
**Goal:** Configure comprehensive telemetry for debugging and driver feedback

**Components:**
- Modify: `src/main/java/frc/robot/subsystems/YAGSLDriveSubsystem.java` - configure YAGSL telemetry
- Modify: `src/main/java/frc/robot/subsystems/VisionSubsystem.java` - add custom telemetry

**Dependencies:** Phase 5 complete (all features implemented)

**Testing:**
- Dashboard shows robot pose on Field2d widget
- Swerve module states visible (angles, velocities)
- Vision telemetry shows tags visible, distance, confidence
- All telemetry updates at expected rates (20ms main, 100ms vision)

**YAGSL Telemetry (in YAGSLDriveSubsystem constructor):**
```java
SwerveDriveTelemetry.verbosity = TelemetryVerbosity.INFO; // Balance visibility and performance
```
- Publishes to `SmartDashboard/swerve/pose`, `SmartDashboard/swerve/modules/`, `SmartDashboard/swerve/gyro/`

**VisionSubsystem Telemetry (add to getPoseEstimate or new periodic method):**
```java
SmartDashboard.putNumber("Vision/TagsVisible", estimate.tagCount);
SmartDashboard.putNumber("Vision/AverageDistance", estimate.avgTagDist);
SmartDashboard.putBoolean("Vision/HasValidPose", estimate.isValid);
SmartDashboard.putNumber("Vision/LastPoseX", estimate.pose.getX());
SmartDashboard.putNumber("Vision/LastPoseY", estimate.pose.getY());
```

**YAGSLDriveSubsystem Telemetry:**
```java
SmartDashboard.putBoolean("Drive/HasAutoLocalized", hasAutoLocalized);
SmartDashboard.putNumber("Drive/VisionLoopRate", visionLoopTimer);
```

**Dashboard Setup:**
- Use Elastic Dashboard or Glass (better swerve support than Shuffleboard)
- Field2d widget showing robot pose + AprilTag positions
- Custom widgets for vision confidence indicators

### Phase 7: Simulation Support Configuration
**Goal:** Configure MapleSim for realistic swerve and vision simulation

**Components:**
- Modify: `src/main/java/frc/robot/subsystems/YAGSLDriveSubsystem.java` - add simulation configuration
- Verify: `vendordeps/maple-sim.json` exists (already present from investigation)

**Dependencies:** Phase 6 complete (telemetry validates simulation behavior)

**Testing:**
- Run robot code in simulation (Robot Simulation GUI)
- Drive robot in sim - observe odometry drift from wheel slippage
- Move near AprilTags - vision should correct drift
- Verify auto-localization triggers in simulation
- Field2d shows accurate pose visualization

**Implementation Details:**
- In YAGSLDriveSubsystem constructor:
  ```java
  if (Robot.isSimulation()) {
      swerveDrive.setHeadingCorrection(false); // Realistic sim behavior
      swerveDrive.setCosineCompensator(false);
  }
  ```
- YAGSL automatically enables MapleSim when in simulation mode
- VisionSubsystem works unchanged - YALL provides simulated MegaTag2 poses in sim

**Simulation Testing Workflow:**
1. Launch simulation (Gradle task: `simulateJava`)
2. Open Glass or AdvantageScope for Field2d visualization
3. Enable robot, drive around field
4. Observe odometry-only drift
5. Drive near AprilTags, observe vision correction
6. Verify auto-localization on first tag detection
7. Tune vision standard deviations based on observed behavior

**MapleSim Features:**
- Realistic 2D physics (dyn4j engine) with friction, collisions
- Simulated encoder drift and skidding
- Vision system provides noisy but accurate poses
- Test vision fusion effectiveness before robot deployment

### Phase 8: Integration and Field Testing
**Goal:** Deploy to robot, validate real-world performance, and tune parameters

**Components:**
- Modify: `src/main/java/frc/robot/RobotContainer.java` - swap old DriveSubsystem for YAGSLDriveSubsystem (comment/uncomment)
- Tune: Vision standard deviation parameters based on field testing

**Dependencies:** Phase 7 complete (simulation validates core functionality)

**Testing:**
- Robot drives accurately with field-relative control
- Auto-localization occurs within 2 seconds of seeing tags
- Autonomous routines complete with <10cm position error
- Pose remains stable across disabled/teleop/auto transitions
- Vision fusion corrects drift during long (15+ second) autonomous paths

**Integration Steps:**
1. Comment out existing DriveSubsystem instantiation in RobotContainer.java:97
2. Instantiate YAGSLDriveSubsystem and VisionSubsystem:
   ```java
   // private final DriveSubsystem m_robotDrive = new DriveSubsystem();
   private final VisionSubsystem m_vision = new VisionSubsystem("limelight");
   private final YAGSLDriveSubsystem m_robotDrive = new YAGSLDriveSubsystem(m_vision);
   ```
3. Update default command to use YAGSLDriveSubsystem.drive() (interface should match)
4. Verify button bindings work (zeroGyro, etc.)

**Tuning Vision Standard Deviations:**
- Drive robot to known positions on field (marked locations)
- Compare reported pose to ground truth
- If vision over-trusted (pose jumps erratically): Increase stddevs
- If vision under-trusted (drift not corrected): Decrease stddevs
- Typical ranges: Multi-tag [0.005-0.05], single tag [0.1-0.5] based on distance

**Performance Validation:**
- Run autonomous path following (PathPlanner)
- Measure final position error with tape measure
- Target: <10cm error on 5+ meter paths
- Vision should prevent accumulating drift over time

**Fallback Plan:**
- If YAGSL has issues during competition, uncomment old DriveSubsystem in RobotContainer
- Code coexists safely for emergency fallback

## Additional Considerations

**Error Handling:**
- Vision estimate failures (no tags visible): Handled gracefully via `Optional<PoseEstimate>` - simply no measurement added
- Limelight disconnection: YALL returns empty Optional, odometry continues with wheel-only estimation
- Invalid pose estimates: Filtered by `isValid` flag and tag count checks before adding measurement
- Notifier thread failures: WPILib Notifier handles exceptions, logs to DriverStation, continues operation

**Edge Cases:**
- Clock skew between RoboRIO and Limelight: MegaTag2 includes timestamp in PoseEstimate, passed directly to addVisionMeasurement()
- Rapid pose changes (spinning robot): Infinite rotation stddev prevents vision from fighting gyro
- Partial AprilTag occlusion: MegaTag2 handles via multi-tag fusion, single-tag estimates get lower trust (higher stddev)
- Match start without visible tags: Robot starts at (0,0) with current gyro heading, auto-localizes once tags appear

**YAGSL 2025 Breaking Changes:**
- Must explicitly call `swerveDrive.zeroGyro()` in autonomous init (no auto-zeroing on startup)
- `physicalproperties.json` format changed to `conversionFactors` (plural) with nested structure
- Absolute encoder feedback requires calling `SwerveDrive.useExternalFeedbackSensor()` if using factor=360

**Hardware Considerations:**
- NEO motors can overheat and short if stalled - ensure proper current limits in SparkFlex/SparkMax config
- Absolute encoder boards must be zip-tied and hot-glued at both connection points (vibration causes disconnects)
- Reduce SparkMax Status Frame 5 to 20ms for accurate odometry (default 200ms causes stale position data)

**Performance Optimization:**
- Vision loop at 100Hz captures all Limelight frames (~90fps for Limelight 3)
- NetworkTables I/O moved to background thread prevents main loop delays
- YAGSL telemetry set to INFO level (not HIGH) to reduce NetworkTables traffic during competition

**Future Extensibility:**
- Multiple Limelight support: Create multiple VisionSubsystem instances, YAGSLDriveSubsystem can pull from array
- Custom field layouts: YALL supports loading custom AprilTag positions for practice fields
- Pose history logging: YAGSL pose can be logged to DataLog for post-match analysis via AdvantageScope
