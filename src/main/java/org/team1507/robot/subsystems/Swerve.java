//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

import org.wpilib.command3.Command;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.util.MathUtil;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;

import org.team1507.lib.core.framework.Subsystem1507;
import org.team1507.lib.core.swerve.SwerveModule1507;
import org.team1507.lib.core.util.Alliance;
import org.team1507.robot.Constants;
import org.team1507.robot.Constants.kSwerve;

import static org.team1507.robot.Constants.kSwerve.kTuning.*;

// ─────────────────────────────────────────────────────────────────────────────
// Swerve
//
// How the drivetrain DRIVES: odometry and pose estimation, driving, headings,
// and every swerve command (teleop, heading control, auto movement).
//
// This robot's drivetrain HARDWARE (CAN IDs, offsets, gear ratios, gains, the
// Tuner X paste zone) lives in SwerveConfig.java.
// ─────────────────────────────────────────────────────────────────────────────
public final class Swerve extends Subsystem1507 {

    // ------------------------------------------------------------
    // Modules
    // ------------------------------------------------------------

    private final SwerveModule1507 frontLeft;
    private final SwerveModule1507 frontRight;
    private final SwerveModule1507 backLeft;
    private final SwerveModule1507 backRight;

    // ------------------------------------------------------------
    // Kinematics & odometry
    // ------------------------------------------------------------

    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;

    // ------------------------------------------------------------
    // Sensors
    // ------------------------------------------------------------

    private final Pigeon2 pigeon;
    private final StatusSignal<Angle> yaw;
    private final StatusSignal<AngularVelocity> yawRate;

    // ------------------------------------------------------------
    // Cached state
    // ------------------------------------------------------------

    private Pose2d pose = new Pose2d();

    private final SwerveModuleVelocity[] moduleStates = new SwerveModuleVelocity[4];
    private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];


    // ------------------------------------------------------------
    // Simulated Data
    // ------------------------------------------------------------

    private double simHeadingRadians = 0.0;
    private ChassisVelocities lastCommandedSpeeds = new ChassisVelocities();

    // ------------------------------------------------------------
    // CAN Signal Batch
    // ------------------------------------------------------------

    private final BaseStatusSignal[] allSignals;

    // ============================================================
    // Constructor
    // ============================================================

    public Swerve() {
        super("Swerve");
        SwerveConfig.checkPastedSettings();
        for (String problem : SwerveConfig.configProblems()) {
            DriverStationErrors.reportWarning("[Swerve config] " + problem, false);
        }

        // Modules, gyro and geometry all come from SwerveConfig (the paste zone).
        this.frontLeft  = SwerveConfig.frontLeft();
        this.frontRight = SwerveConfig.frontRight();
        this.backLeft   = SwerveConfig.backLeft();
        this.backRight  = SwerveConfig.backRight();
        this.kinematics = SwerveConfig.kinematics();
        this.pigeon     = new Pigeon2(SwerveConfig.pigeonId(), Constants.CAN_BUS);

        // Count all 8 motors toward Swerve/TotalSupplyCurrent (logged every loop
        // by Subsystem1507). Swerve refreshes them itself in periodic(), together
        // with the CANcoders and gyro, so they are tracked, not managed.
        trackMotors(
            frontLeft.getDriveMotor(),  frontLeft.getSteerMotor(),
            frontRight.getDriveMotor(), frontRight.getSteerMotor(),
            backLeft.getDriveMotor(),   backLeft.getSteerMotor(),
            backRight.getDriveMotor(),  backRight.getSteerMotor()
        );

        // (false) = don't read yet: the Pigeon may still be booting, and an early
        // read only prints a "CAN frame not received" error. periodic() refreshes them.
        this.yaw = pigeon.getYaw(false);
        this.yawRate = pigeon.getAngularVelocityZWorld(false);

        // The heading feeds odometry and field-relative driving, so it must update
        // every loop. Without this, optimizeBusUtilizationForAll() below would drop
        // yaw to 4 Hz (it slows every signal that has no explicit frequency).
        BaseStatusSignal.setUpdateFrequencyForAll(100.0, yaw, yawRate);

        this.poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            getGyroHeading(),
            getModulePositions(),
            pose,
            kSwerve.kTuning.ODOMETRY_STD_DEV,
            kSwerve.kTuning.VISION_STD_DEV
        );

        // Disable unused CAN signals on every swerve device to avoid flooding the bus.
        // Only signals given an explicit frequency above keep their rate.
        ParentDevice.optimizeBusUtilizationForAll(
            4.0,
            frontLeft.getDriveDevice(),  frontLeft.getSteerDevice(),  frontLeft.getEncoderDevice(),
            frontRight.getDriveDevice(), frontRight.getSteerDevice(), frontRight.getEncoderDevice(),
            backLeft.getDriveDevice(),   backLeft.getSteerDevice(),   backLeft.getEncoderDevice(),
            backRight.getDriveDevice(),  backRight.getSteerDevice(),  backRight.getEncoderDevice(),
            pigeon
        );

        // Build the signal array once: every module's signals + Pigeon2 yaw and yaw
        // rate. periodic() refreshes all of them in a single CAN call.
        List<BaseStatusSignal> signals = new ArrayList<>();
        signals.addAll(List.of(frontLeft.getAllSignals()));
        signals.addAll(List.of(frontRight.getAllSignals()));
        signals.addAll(List.of(backLeft.getAllSignals()));
        signals.addAll(List.of(backRight.getAllSignals()));
        signals.add(yaw);
        signals.add(yawRate);
        allSignals = signals.toArray(BaseStatusSignal[]::new);
    }


    // ============================================================
    // Periodic
    // ============================================================

    @Override
    public void periodic() {
        if (RobotBase.isSimulation()) {
            // Integrate heading before poseEstimator.update() so the heading is fresh.
            // getGyroHeading() reads simHeadingRadians directly in sim — no Pigeon signal needed.
            simHeadingRadians += lastCommandedSpeeds.omega * 0.02;
        } else {
            BaseStatusSignal.refreshAll(allSignals);
        }

        pose = poseEstimator.update(
            getGyroHeading(),
            getModulePositions()
        );

        log("Pose", pose);
        log("DriveStalled", isAnyDriveStalled());
        log("SteerStalled", isAnySteerStalled());
        log("ModuleStates", getModuleStates());
    }

    @Override
    public void simulationPeriodic() {
        frontLeft.simulationUpdate(0.02);
        frontRight.simulationUpdate(0.02);
        backLeft.simulationUpdate(0.02);
        backRight.simulationUpdate(0.02);
    }

    // ============================================================
    // Low-Level Control
    //
    // These are the raw driving methods called by commands.
    // Students generally don't call these directly — use a command.
    // ============================================================

    /**
     * Drives using ROBOT-relative ChassisVelocities (x = robot forward, y = robot left).
     * The kinematics layer converts these to individual module states.
     *
     * <p>To drive in field or driver directions, convert first:
     * {@code velocities.toRobotRelative(getPose().getRotation())} (field) or
     * {@code velocities.toRobotRelative(getDriverRelativeHeading())} (driver).
     * Every command in this file does that before calling drive().
     *
     * <p>{@code ChassisVelocities.discretize()} is applied before kinematics to correct
     * for the skew that occurs when the robot translates and rotates simultaneously
     * within a single 20 ms loop iteration. Without it, the robot arcs instead of
     * driving in a straight line.
     */
    public void drive(ChassisVelocities speeds) {
        lastCommandedSpeeds = speeds;
        SwerveModuleVelocity[] states = kinematics.toSwerveModuleVelocities(
            speeds.discretize(0.02)
        );
        // 2027: desaturateWheelVelocities returns a NEW array (it no longer edits
        // the one passed in), so the result must be assigned back.
        states = SwerveDriveKinematics.desaturateWheelVelocities(states, SwerveConfig.MAX_SPEED);

        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    /** Stops all modules immediately. */
    public void stop() {
        lastCommandedSpeeds = new ChassisVelocities();
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /**
     * Sets all modules to the X configuration (FL/BR at +45°, FR/BL at -45°).
     *
     * <p>Uses {@code brakeToAngle()} on each module rather than
     * {@code setDesiredState()} so the steer angle is applied unconditionally.
     * {@code setDesiredState()} has an anti-jitter guard that skips steer
     * updates when speed is near zero — which would prevent the X pattern
     * from ever forming since brake() always passes speed = 0.
     *
     * <p>Called every loop by {@link #brakeCommand()} to hold the configuration.
     */
    public void brake() {
        frontLeft.brakeToAngle(Rotation2d.fromDegrees( 45));
        frontRight.brakeToAngle(Rotation2d.fromDegrees(-45));
        backLeft.brakeToAngle(Rotation2d.fromDegrees(-45));
        backRight.brakeToAngle(Rotation2d.fromDegrees( 45));
    }

    // ============================================================
    // Observation
    // ============================================================

    /** Returns the robot's current estimated field pose (Blue-origin field coordinates). */
    public Pose2d getPose() {
        return pose;
    }

    /**
     * The field direction the driver calls "forward": straight away from their
     * own driver station. 0° on Blue, 180° on Red (field coordinates are always
     * Blue-origin). Blue when the alliance is unknown (practice, simulation).
     */
    private static Rotation2d driverForward() {
        return Alliance.isRed() ? Rotation2d.fromDegrees(180.0) : Rotation2d.ZERO;
    }

    /**
     * The robot's heading as the DRIVER sees it: 0° = facing straight away from
     * the driver, on either alliance. Use this for driver-relative (field-oriented)
     * teleop driving.
     *
     * <p>It comes from the pose estimate, so when vision corrects the pose,
     * driving corrects with it.
     */
    public Rotation2d getDriverRelativeHeading() {
        return getPose().getRotation().minus(driverForward());
    }

    /**
     * The raw gyro heading from the Pigeon2 (or simulated equivalent).
     *
     * <p>This is NOT the field heading: it starts wherever the robot was pointing
     * at power-on. The pose estimator turns it into a field heading. Use
     * {@code getPose().getRotation()} for the field heading, or
     * {@link #getDriverRelativeHeading()} for teleop driving.
     */
    public Rotation2d getGyroHeading() {
        if (RobotBase.isSimulation()) {
            return Rotation2d.fromRadians(simHeadingRadians);
        }
        // Latency-compensated: projects yaw forward by yaw rate x signal age.
        return Rotation2d.fromDegrees(BaseStatusSignal.getLatencyCompensatedValueAsDouble(yaw, yawRate));
    }

    /** Returns the robot's current robot-relative chassis speeds derived from module states. */
    public ChassisVelocities getChassisVelocities() {
        return kinematics.toChassisVelocities(getModuleStates());
    }

    /**
     * Returns the robot's velocity in field-relative coordinates.
     * Used by maintainHeadingToTarget for motion compensation.
     */
    public ChassisVelocities getFieldRelativeSpeeds() {
        return kinematics.toChassisVelocities(getModuleStates())
            .toFieldRelative(getPose().getRotation());
    }

    /** Returns the configured maximum translational speed in m/s. */
    public double getMaxSpeed() {
        return SwerveConfig.MAX_SPEED;
    }

    /** Returns the configured maximum angular rate in rad/s. */
    public double getMaxAngular() {
        return SwerveConfig.MAX_ANGULAR_RATE;
    }

    // ============================================================
    // Vision & Localization
    // ============================================================

    /**
     * Feeds a vision-estimated pose into the pose estimator.
     *
     * @param visionPose        estimated robot pose from vision
     * @param timestampSeconds  robot timestamp (Timer.getTimestamp()) when the measurement was captured
     * @param stdDevs           measurement confidence [x (m), y (m), heading (rad)]
     */
    public void addVisionMeasurement(
        Pose2d visionPose,
        double timestampSeconds,
        Matrix<N3, N1> stdDevs
    ) {
        poseEstimator.addVisionMeasurement(
            visionPose,
            timestampSeconds,
            stdDevs
        );
    }

    /**
     * Tells the robot it is facing straight away from the driver.
     *
     * <p>Sets the pose estimator's FIELD heading to the driver's forward direction:
     * 0° on Blue, 180° on Red. The robot's position is kept. The raw gyro is not
     * touched; the pose estimator handles the offset.
     *
     * <p>(Until 2027 this set the heading to 0° on both alliances, which made the
     * field pose 180° wrong on Red and confused auto commands and vision.)
     *
     * <p>With QuestNav/AprilTag vision correcting the pose, this is only a fallback
     * for when vision isn't available.
     */
    public void zeroHeading() {
        resetPose(new Pose2d(getPose().getTranslation(), driverForward()));
    }

    /**
     * Resets the robot's pose estimate to a known field position.
     * Used by autonomous routines and by QuestNavSubsystem after a confirmed pose reset.
     *
     * @param pose the new known pose
     */
    public void resetPose(Pose2d pose) {
        this.pose = pose;
        poseEstimator.resetPosition(
            getGyroHeading(),
            getModulePositions(),
            pose
        );
    }

    // ============================================================
    // Fault interpretation
    // ============================================================

    /** Returns true if any drive motor is currently reporting a stall condition. */
    public boolean isAnyDriveStalled() {
        return frontLeft.isDriveStalled()
            || frontRight.isDriveStalled()
            || backLeft.isDriveStalled()
            || backRight.isDriveStalled();
    }

    /** Returns true if any steer motor is currently reporting a stall condition. */
    public boolean isAnySteerStalled() {
        return frontLeft.isSteerStalled()
            || frontRight.isSteerStalled()
            || backLeft.isSteerStalled()
            || backRight.isSteerStalled();
    }

    // ============================================================
    // Internal Helpers
    //
    // Private math utilities shared across multiple commands.
    // ============================================================

    /**
     * Simple proportional heading controller.
     * Returns an angular velocity (rad/s) to steer from current → desired rotation.
     *
     * <p>The error is wrapped to [-π, π] so the robot always takes the shortest path.
     * Output is clamped to the robot's maximum angular rate so large errors (e.g. 180°)
     * cannot produce physically impossible rotation commands.
     *
     * <p>HEADING_KP (5.5) means 1 radian of error produces 5.5 rad/s of correction.
     * Without clamping, a 180° error would produce ~17.3 rad/s — above the ~13.1 rad/s
     * physical maximum.
     */
    private double computeOmega(Rotation2d current, Rotation2d desired) {
        double error = MathUtil.angleModulus(desired.minus(current).getRadians());
        return Math.clamp(error * HEADING_KP, -SwerveConfig.MAX_ANGULAR_RATE, SwerveConfig.MAX_ANGULAR_RATE);
    }

    /**
     * Computes the direction the robot must face to look directly at a target.
     * Returns a Rotation2d pointing from robot's current XY → target's XY.
     * The target's own rotation field is ignored.
     */
    private Rotation2d computeHeadingToTarget(Pose2d robot, Pose2d target) {
        double dx = target.getX() - robot.getX();
        double dy = target.getY() - robot.getY();
        return new Rotation2d(Math.atan2(dy, dx));
    }

    private SwerveModuleVelocity[] getModuleStates() {
        moduleStates[0] = frontLeft.getState();
        moduleStates[1] = frontRight.getState();
        moduleStates[2] = backLeft.getState();
        moduleStates[3] = backRight.getState();
        return moduleStates;
    }

    private SwerveModulePosition[] getModulePositions() {
        modulePositions[0] = frontLeft.getPosition();
        modulePositions[1] = frontRight.getPosition();
        modulePositions[2] = backLeft.getPosition();
        modulePositions[3] = backRight.getPosition();
        return modulePositions;
    }

    // ╔══════════════════════════════════════════════════════════════════╗
    // ║                        COMMANDS (Commands v3)                    ║
    // ║                                                                  ║
    // ║  All swerve commands live here, directly on the subsystem.       ║
    // ║  run(...) and runRepeatedly(...) come from Mechanism and make    ║
    // ║  this subsystem the command's requirement automatically.         ║
    // ║                                                                  ║
    // ║  How a v3 command reads, top to bottom:                          ║
    // ║                                                                  ║
    // ║    return run(coroutine -> {                                     ║
    // ║        // setup: runs once when the command starts               ║
    // ║        while (!done()) {                                         ║
    // ║            drive(...);          // runs every loop               ║
    // ║            coroutine.yield();   // REQUIRED in every loop        ║
    // ║        }                                                         ║
    // ║        stop();                  // runs when it finishes         ║
    // ║    })                                                            ║
    // ║    .whenCanceled(this::stop)    // runs if it is interrupted     ║
    // ║    .named("Swerve.myCommand");  // every command needs a name    ║
    // ║                                                                  ║
    // ║  Forgetting coroutine.yield() inside a loop freezes the whole    ║
    // ║  robot program. For "do this every loop until interrupted",      ║
    // ║  use runRepeatedly(() -> ...), which yields for you.             ║
    // ║                                                                  ║
    // ║  SECTIONS:                                                       ║
    // ║    1. Basic Drive (teleop)                                       ║
    // ║    2. Heading Control (pointing, aiming)                         ║
    // ║    3. Autonomous Movement (driveToPoint, moveThroughPose, etc.)  ║
    // ║    4. Utility (brake)                                            ║
    // ╚══════════════════════════════════════════════════════════════════╝


    // ─────────────────────────────────────────────────────────────────
    // 1. BASIC DRIVE
    // ─────────────────────────────────────────────────────────────────

    /**
     * What the drivetrain does when no other command is using it: stop the
     * modules and hold. Robot.java sets this as the default command, and
     * DriverTeleop replaces it with joystick driving while teleop is selected.
     */
    @Override
    public Command idle() {
        return run(coroutine -> {
            stop();
            coroutine.park(); // stay "running" (and stopped) until something else needs swerve
        })
        .withPriority(Command.LOWEST_PRIORITY)
        .named("Swerve.idle");
    }

    /**
     * Teleop drive command.
     * Accepts a ChassisVelocities supplier so the OpMode can compute velocities
     * from controller inputs each loop iteration.
     *
     *   swerve.setDefaultCommand(swerve.driveCommand(() -> computeVelocities()));
     */
    public Command driveCommand(Supplier<ChassisVelocities> velocities) {
        return runRepeatedly(() -> drive(velocities.get()))
            .whenCanceled(this::stop)
            .named("Swerve.drive");
    }

    /**
     * Drives at fixed ChassisVelocities until interrupted. Used by auto commands.
     */
    public Command driveCommand(ChassisVelocities velocities) {
        return runRepeatedly(() -> drive(velocities))
            .whenCanceled(this::stop)
            .named("Swerve.driveFixed");
    }

    /**
     * Drives at the given ChassisVelocities for a fixed number of seconds, then stops.
     */
    public Command driveForTime(ChassisVelocities velocities, double seconds) {
        return run(coroutine -> {
            double endTime = Timer.getTimestamp() + seconds;
            while (Timer.getTimestamp() < endTime) {
                drive(velocities);
                coroutine.yield();
            }
            stop();
        })
        .whenCanceled(this::stop)
        .named("Swerve.driveForTime");
    }

    /** Stops all modules. Finishes immediately. */
    public Command stopCommand() {
        return run(coroutine -> stop())
            .named("Swerve.stop");
    }

    /**
     * Tells the robot it is facing straight away from the driver (see zeroHeading()).
     * Bound to the bottom face button in DriverTeleop.
     */
    public Command zeroHeadingCommand() {
        return run(coroutine -> zeroHeading())
            .named("Swerve.zeroHeading");
    }

    /** Resets the robot's pose estimate to a given field position. Finishes immediately. */
    public Command resetPoseCommand(Pose2d pose) {
        return run(coroutine -> resetPose(pose))
            .named("Swerve.resetPose");
    }


    // ─────────────────────────────────────────────────────────────────
    // 2. HEADING CONTROL
    // ─────────────────────────────────────────────────────────────────

    /** True when the robot's heading is within HEADING_TOLERANCE_DEG of the desired heading. */
    private boolean isFacing(Rotation2d desired) {
        return Math.abs(MathUtil.angleModulus(
            getPose().getRotation().minus(desired).getRadians()
        )) < Math.toRadians(HEADING_TOLERANCE_DEG);
    }

    /**
     * Rotates the robot in place until it faces a fixed field position.
     *
     * The robot looks toward the XY of targetPose — the target's own rotation
     * is ignored. Finishes within HEADING_TOLERANCE_DEG.
     *
     * Use case: snap to face the speaker/hub before shooting.
     *
     * @param targetPose  the field position to face toward
     */
    public Command pointToTarget(Pose2d targetPose) {
        return pointToTarget(() -> targetPose, "Swerve.pointToTarget");
    }

    /**
     * Rotates the robot in place to face a dynamic target.
     * The target pose is re-read from the supplier every loop.
     * Finishes within HEADING_TOLERANCE_DEG.
     *
     * Use case: track a moving vision target or live shooter setpoint.
     *
     * @param targetPoseSupplier  supplier that returns the current target pose
     */
    public Command pointToTarget(Supplier<Pose2d> targetPoseSupplier) {
        return pointToTarget(targetPoseSupplier, "Swerve.pointToTargetDynamic");
    }

    private Command pointToTarget(Supplier<Pose2d> targetPoseSupplier, String name) {
        return turnToFace(() -> computeHeadingToTarget(getPose(), targetPoseSupplier.get()), name);
    }

    /**
     * Rotates the robot in place to match a specific heading in degrees.
     *
     * Finishes within HEADING_TOLERANCE_DEG.
     *
     * @param angleDeg  desired heading in degrees (e.g. 90 = left, 180 = backwards)
     */
    public Command changeHeading(double angleDeg) {
        return changeHeading(Rotation2d.fromDegrees(angleDeg));
    }

    /**
     * Rotates the robot in place to match the given Rotation2d.
     *
     * Differs from pointToTarget: this snaps to a specific angle,
     * not a direction toward a field location. Useful for correcting heading
     * drift after moveThroughPose, or for aligning with a wall.
     *
     * Finishes within HEADING_TOLERANCE_DEG.
     *
     * @param targetHeading  the desired heading
     */
    public Command changeHeading(Rotation2d targetHeading) {
        return turnToFace(() -> targetHeading, "Swerve.changeHeading");
    }

    /**
     * Rotates the robot in place to match the heading stored in a target Pose2d,
     * ignoring that pose's XY position.
     *
     * @param targetPose  the pose whose .getRotation() defines the desired heading
     */
    public Command changeHeading(Pose2d targetPose) {
        return changeHeading(targetPose.getRotation());
    }

    /**
     * Drives with driver-supplied translation while automatically aiming at a target.
     *
     * Includes motion compensation — the aim leads the robot's velocity so projectiles
     * arrive correctly even while moving. Tune AIM_LEAD_TIME in Constants.kSwerve.kTuning.
     *
     * This command NEVER finishes on its own. Bind with whileTrue() so it cancels
     * when the button is released, returning control to the default drive command.
     *
     * Use case: driver holds a button to auto-aim while keeping full translation control.
     *
     * @param targetPoseSupplier  field target to aim at (re-read every loop)
     * @param xSupplier           driver forward/back input (m/s, driver-relative: + = away from driver)
     * @param ySupplier           driver strafe input (m/s, driver-relative: + = driver's left)
     */
    public Command maintainHeadingToTarget(
        Supplier<Pose2d> targetPoseSupplier,
        Supplier<Double> xSupplier,
        Supplier<Double> ySupplier
    ) {
        return runRepeatedly(() -> {
            Pose2d currentPose  = getPose();
            Pose2d targetPose   = targetPoseSupplier.get();
            ChassisVelocities field = getFieldRelativeSpeeds();

            // Shift the aim point forward in time to compensate for robot motion.
            // If the robot moves right at 2 m/s and lead time is 0.25 s,
            // the compensated target shifts 0.5 m right — correcting for
            // the time the game piece takes to travel to the target.
            Translation2d compensated = targetPose.getTranslation().minus(
                new Translation2d(
                    field.vx * AIM_LEAD_TIME,
                    field.vy * AIM_LEAD_TIME
                )
            );

            Rotation2d desiredHeading = compensated
                .minus(currentPose.getTranslation())
                .getAngle()
                // 2027: getAngle() is empty for a zero-length vector (robot exactly
                // on the target). Keep the current heading in that case.
                .orElse(currentPose.getRotation());

            double omega = computeOmega(currentPose.getRotation(), desiredHeading);
            // xSupplier / ySupplier are driver-relative — convert to robot-relative
            // before passing to drive() so translation is correct at any heading,
            // on either alliance.
            drive(new ChassisVelocities(
                xSupplier.get(), ySupplier.get(), omega
            ).toRobotRelative(getDriverRelativeHeading()));
        })
        .whenCanceled(this::stop)
        .named("Swerve.maintainHeadingToTarget");
    }


    // ─────────────────────────────────────────────────────────────────
    // 3. AUTONOMOUS MOVEMENT
    // ─────────────────────────────────────────────────────────────────

    /**
     * Drives toward a target pose and finishes within ARRIVE_THRESHOLD (5 cm)
     * of the target XY. Slows down near the target (APF deceleration ramp).
     *
     * Rotation toward the target pose's heading is corrected proportionally
     * throughout the move.
     *
     * Stall detection: if the robot moves less than STALL_THRESHOLD for
     * STALL_TIMEOUT seconds (e.g. pushed against a wall), the command gives up
     * so the auto can continue.
     *
     * @param targetPose  the field pose to drive to
     * @param velocity    cruise translation speed (m/s)
     * @param stopAtEnd   true = stop when done; false = leave velocity applied (for chaining)
     */
    public Command driveToPoint(Pose2d targetPose, double velocity, boolean stopAtEnd) {
        return driveToTarget(start -> targetPose, velocity, stopAtEnd, "Swerve.driveToPoint");
    }

    /**
     * Drives through a waypoint at constant speed without slowing down.
     *
     * Unlike driveToPoint, the command finishes as soon as the robot enters the
     * pass radius — there is no deceleration. This allows smooth, high-speed
     * paths through multiple chained waypoints.
     *
     * Rotation is controlled by a PID toward the target pose's heading.
     * Tune THETA_KP / KI / KD and MOVE_THROUGH_DEFAULT_RADIUS in Constants.kSwerve.kTuning.
     *
     * Gives up (so the auto can continue) if the robot stalls for STALL_TIMEOUT
     * seconds, or runs longer than MAX_MOVETHROUGH_SECONDS in total.
     *
     * @param targetPose  the waypoint pose to pass through
     * @param maxSpeed    translation speed (m/s)
     * @param maxAngular  maximum rotation speed (rad/s), clamps PID output
     * @param passRadius  distance to consider the waypoint "passed" (meters)
     */
    public Command moveThroughPose(
        Pose2d targetPose,
        double maxSpeed,
        double maxAngular,
        double passRadius
    ) {
        return run(coroutine -> {
            PIDController thetaPID = new PIDController(THETA_KP, THETA_KI, THETA_KD);
            thetaPID.enableContinuousInput(-Math.PI, Math.PI);

            double startTime = Timer.getTimestamp();
            ProgressWatchdog watchdog = new ProgressWatchdog(getPose().getTranslation());

            while (true) {
                Pose2d current = getPose();

                // Done: entered the pass radius, stalled, or hit the hard time limit
                // (the time limit catches oscillation that keeps resetting the watchdog)
                if (current.getTranslation().getDistance(targetPose.getTranslation()) < passRadius
                        || watchdog.isStalled(current.getTranslation())
                        || Timer.getTimestamp() - startTime > MAX_MOVETHROUGH_SECONDS) {
                    break;
                }

                // Normalized direction toward the waypoint, at constant speed
                double dx       = targetPose.getX() - current.getX();
                double dy       = targetPose.getY() - current.getY();
                double distance = Math.hypot(dx, dy);

                // PID rotation toward target heading, clamped to maxAngular
                double omega = Math.clamp(
                    thetaPID.calculate(
                        current.getRotation().getRadians(),
                        targetPose.getRotation().getRadians()
                    ),
                    -maxAngular, maxAngular
                );

                drive(new ChassisVelocities(
                    dx / distance * maxSpeed, dy / distance * maxSpeed, omega
                ).toRobotRelative(current.getRotation()));

                coroutine.yield();
            }
            stop();
        })
        .whenCanceled(this::stop)
        .named("Swerve.moveThroughPose");
    }

    /**
     * Convenience overload of moveThroughPose using the default pass radius.
     * Use this when you don't need to tune the radius per waypoint.
     */
    public Command moveThroughPose(Pose2d targetPose, double maxSpeed, double maxAngular) {
        return moveThroughPose(targetPose, maxSpeed, maxAngular, MOVE_THROUGH_DEFAULT_RADIUS);
    }

    /**
     * Drives forward a fixed distance along the robot's current heading.
     *
     * The target is computed from the robot's position and heading at the moment
     * the command starts. Heading is held with proportional correction throughout.
     * Finishes within ARRIVE_THRESHOLD (5 cm).
     *
     * Use case: fallback auto when vision is offline — just drive a known distance.
     *
     * @param distanceMeters  distance to drive (meters, positive = forward)
     * @param velocity        cruise translation speed (m/s)
     * @param stopAtEnd       true = stop when done
     */
    public Command driveForwardMeters(double distanceMeters, double velocity, boolean stopAtEnd) {
        return driveToTarget(start -> {
            Rotation2d heading = start.getRotation();
            Translation2d offset = new Translation2d(
                distanceMeters * heading.getCos(), distanceMeters * heading.getSin());
            return new Pose2d(start.getTranslation().plus(offset), heading);
        }, velocity, stopAtEnd, "Swerve.driveForwardMeters");
    }


    // ─────────────────────────────────────────────────────────────────
    // SHARED COMMAND BUILDING BLOCKS
    //
    // Private helpers used by the commands above, so each piece of logic
    // exists in exactly one place.
    // ─────────────────────────────────────────────────────────────────

    /**
     * Turns in place until the robot faces {@code desiredHeading} (re-read
     * every loop), within HEADING_TOLERANCE_DEG, then stops.
     * Used by pointToTarget and changeHeading.
     */
    private Command turnToFace(Supplier<Rotation2d> desiredHeading, String name) {
        return run(coroutine -> {
            Rotation2d desired = desiredHeading.get();
            while (!isFacing(desired)) {
                drive(new ChassisVelocities(0.0, 0.0, computeOmega(getPose().getRotation(), desired)));
                coroutine.yield();
                desired = desiredHeading.get();
            }
            stop();
        })
        .whenCanceled(this::stop)
        .named(name);
    }

    /**
     * Drives to a target pose, slowing down as it gets close (APF deceleration
     * ramp) while turning toward the target's heading. Finishes within
     * ARRIVE_THRESHOLD, or gives up if the robot stops making progress (see
     * {@link ProgressWatchdog}) so an auto can continue.
     * Used by driveToPoint and driveForwardMeters.
     *
     * @param targetFromStart works out the target from the robot's pose when the
     *                        command STARTS (driveForwardMeters needs this)
     */
    private Command driveToTarget(
        Function<Pose2d, Pose2d> targetFromStart, double velocity, boolean stopAtEnd, String name
    ) {
        return run(coroutine -> {
            Pose2d target = targetFromStart.apply(getPose());
            ProgressWatchdog watchdog = new ProgressWatchdog(getPose().getTranslation());

            while (true) {
                Pose2d current = getPose();
                Translation2d toTarget = target.getTranslation().minus(current.getTranslation());
                double distance = toTarget.getNorm();

                // Done: arrived, or stalled (pushed against a wall, stuck on something)
                if (distance < ARRIVE_THRESHOLD || watchdog.isStalled(current.getTranslation())) {
                    break;
                }

                // Drive toward the target, slowing down as it gets close
                double speed = Math.min(ARRIVE_KP * distance, velocity);
                drive(new ChassisVelocities(
                    toTarget.getX() / distance * speed,
                    toTarget.getY() / distance * speed,
                    computeOmega(current.getRotation(), target.getRotation())
                ).toRobotRelative(current.getRotation()));

                coroutine.yield();
            }

            if (stopAtEnd) stop();
        })
        .whenCanceled(() -> { if (stopAtEnd) stop(); })
        .named(name);
    }

    /**
     * Tells a driving command when the robot has stopped making progress:
     * it moved less than STALL_THRESHOLD meters in the last STALL_TIMEOUT
     * seconds (pushed against a wall, stuck on a game piece). Each command
     * run creates its own watchdog when it starts.
     */
    private static final class ProgressWatchdog {
        private Translation2d lastProgressPoint;
        private double lastProgressTime;

        ProgressWatchdog(Translation2d start) {
            lastProgressPoint = start;
            lastProgressTime = Timer.getTimestamp();
        }

        /** Call once per loop with the robot's position; true once it has stalled. */
        boolean isStalled(Translation2d position) {
            double now = Timer.getTimestamp();
            if (position.getDistance(lastProgressPoint) > STALL_THRESHOLD) {
                lastProgressPoint = position;
                lastProgressTime = now;
                return false;
            }
            return now - lastProgressTime > STALL_TIMEOUT;
        }
    }


    // ─────────────────────────────────────────────────────────────────
    // 4. UTILITY
    // ─────────────────────────────────────────────────────────────────

    /**
     * Brakes all modules in the X configuration to resist pushing.
     *
     * FL and BR point to +45°, FR and BL point to -45°. The crossed pattern
     * makes it very hard to push the robot out of position.
     *
     * The command runs continuously and must be interrupted to exit.
     * Bind with whileTrue() so the brake releases when the button is released.
     *
     * Use case: hold position while shooting, resist defense, stay on a slope.
     */
    public Command brakeCommand() {
        return runRepeatedly(this::brake)
            .whenCanceled(this::stop)
            .named("Swerve.brake");
    }
}
