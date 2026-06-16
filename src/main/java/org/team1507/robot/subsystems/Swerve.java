//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;

import org.team1507.lib.core.framework.Subsystem1507;
import org.team1507.lib.core.impl.ctre.Motor1507;
import org.team1507.lib.core.logging.Telemetry;
import org.team1507.lib.core.swerve.SwerveModule1507;
import org.team1507.lib.core.swerve.SwerveModule1507.MathConfig;
import org.team1507.robot.Constants.RobotMap;
import org.team1507.robot.Constants.kSwerve;
import static org.team1507.robot.Constants.kSwerve.kTuning.*;

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

    // ------------------------------------------------------------
    // Cached state
    // ------------------------------------------------------------

    private Pose2d pose = new Pose2d();

    private final SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    private final double maxSpeedMetersPerSecond;
    private final double maxAngularMetersPerSecond;


    // ------------------------------------------------------------
    // Simulated Data
    // ------------------------------------------------------------

    private double simHeadingRadians = 0.0;
    private ChassisSpeeds lastCommandedSpeeds = new ChassisSpeeds();

    // ------------------------------------------------------------
    // CAN Signal Batch
    // ------------------------------------------------------------

    private final BaseStatusSignal[] allSignals;

    // ============================================================
    // Constructor
    // ============================================================

    public Swerve() {
        super("Swerve");
        MathConfig math = new MathConfig(
            kSwerve.DRIVE_GEAR_RATIO, kSwerve.STEER_GEAR_RATIO,
            kSwerve.COUPLE_RATIO, kSwerve.WHEEL_RADIUS_METERS
        );

        this.frontLeft = new SwerveModule1507("FrontLeft",
            new Motor1507("Swerve/FrontLeft/Drive", Motor1507.Type.FX, RobotMap.FL_DRIVE, kSwerve.FRONT_LEFT_DRIVE_CONFIG),
            new Motor1507("Swerve/FrontLeft/Steer", Motor1507.Type.FX, RobotMap.FL_STEER, kSwerve.FRONT_LEFT_STEER_CONFIG),
            new CANcoder(RobotMap.FL_ENCODER),
            new Rotation2d(Rotations.of(RobotMap.FL_ENCODER_OFFSET)),
            math, kSwerve.DRIVE_METERS_SCALE
        );

        this.frontRight = new SwerveModule1507("FrontRight",
            new Motor1507("Swerve/FrontRight/Drive", Motor1507.Type.FX, RobotMap.FR_DRIVE, kSwerve.FRONT_RIGHT_DRIVE_CONFIG),
            new Motor1507("Swerve/FrontRight/Steer", Motor1507.Type.FX, RobotMap.FR_STEER, kSwerve.FRONT_RIGHT_STEER_CONFIG),
            new CANcoder(RobotMap.FR_ENCODER),
            new Rotation2d(Rotations.of(RobotMap.FR_ENCODER_OFFSET)),
            math, kSwerve.DRIVE_METERS_SCALE
        );

        this.backLeft = new SwerveModule1507("BackLeft",
            new Motor1507("Swerve/BackLeft/Drive", Motor1507.Type.FX, RobotMap.BL_DRIVE, kSwerve.BACK_LEFT_DRIVE_CONFIG),
            new Motor1507("Swerve/BackLeft/Steer", Motor1507.Type.FX, RobotMap.BL_STEER, kSwerve.BACK_LEFT_STEER_CONFIG),
            new CANcoder(RobotMap.BL_ENCODER),
            new Rotation2d(Rotations.of(RobotMap.BL_ENCODER_OFFSET)),
            math, kSwerve.DRIVE_METERS_SCALE
        );

        this.backRight = new SwerveModule1507("BackRight",
            new Motor1507("Swerve/BackRight/Drive", Motor1507.Type.FX, RobotMap.BR_DRIVE, kSwerve.BACK_RIGHT_DRIVE_CONFIG),
            new Motor1507("Swerve/BackRight/Steer", Motor1507.Type.FX, RobotMap.BR_STEER, kSwerve.BACK_RIGHT_STEER_CONFIG),
            new CANcoder(RobotMap.BR_ENCODER),
            new Rotation2d(Rotations.of(RobotMap.BR_ENCODER_OFFSET)),
            math, kSwerve.DRIVE_METERS_SCALE
        );

        this.pigeon = new Pigeon2(RobotMap.PIGEON2);
        this.maxSpeedMetersPerSecond = kSwerve.MAX_SPEED;
        this.maxAngularMetersPerSecond = kSwerve.MAX_ANGULAR_RATE;

        this.yaw = pigeon.getYaw();

        this.kinematics = new SwerveDriveKinematics(
            kSwerve.FRONT_LEFT_LOCATION, kSwerve.FRONT_RIGHT_LOCATION,
            kSwerve.BACK_LEFT_LOCATION,  kSwerve.BACK_RIGHT_LOCATION
        );

        this.poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            getHeading(),
            getModulePositions(),
            pose,
            kSwerve.ODOMETRY_STD_DEV,
            kSwerve.VISION_STD_DEV
        );

        // Disable unused CAN signals on every swerve device to avoid flooding the bus.
        ParentDevice.optimizeBusUtilizationForAll(
            4.0,
            frontLeft.getDriveDevice(),  frontLeft.getSteerDevice(),  frontLeft.getEncoderDevice(),
            frontRight.getDriveDevice(), frontRight.getSteerDevice(), frontRight.getEncoderDevice(),
            backLeft.getDriveDevice(),   backLeft.getSteerDevice(),   backLeft.getEncoderDevice(),
            backRight.getDriveDevice(),  backRight.getSteerDevice(),  backRight.getEncoderDevice(),
            pigeon
        );

        // Build the master signal array once — 4 modules × 14 signals + 1 Pigeon2 yaw = 57
        allSignals = new BaseStatusSignal[57];
        int idx = 0;
        for (BaseStatusSignal s : frontLeft.getAllSignals())  allSignals[idx++] = s;
        for (BaseStatusSignal s : frontRight.getAllSignals()) allSignals[idx++] = s;
        for (BaseStatusSignal s : backLeft.getAllSignals())   allSignals[idx++] = s;
        for (BaseStatusSignal s : backRight.getAllSignals())  allSignals[idx++] = s;
        allSignals[idx] = yaw;

        Telemetry.set("Swerve/Initialized", true);
    }

    // ============================================================
    // Periodic
    // ============================================================

    @Override
    public void periodic() {
        if (RobotBase.isSimulation()) {
            // Integrate heading before poseEstimator.update() so the heading is fresh.
            // getHeading() reads simHeadingRadians directly in sim — no Pigeon signal needed.
            simHeadingRadians += lastCommandedSpeeds.omegaRadiansPerSecond * 0.02;
        } else {
            BaseStatusSignal.refreshAll(allSignals);
        }

        pose = poseEstimator.update(
            getHeading(),
            getModulePositions()
        );

        log("Pose", pose);
        log("DriveStalled", isAnyDriveStalled());
        log("SteerStalled", isAnySteerStalled());
        Telemetry.set("Swerve/ModuleStates", getModuleStates());
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
     * Drives using field-relative ChassisSpeeds (x = forward, y = left).
     * The kinematics layer converts these to individual module states.
     *
     * <p>{@code ChassisSpeeds.discretize()} is applied before kinematics to correct
     * for the skew that occurs when the robot translates and rotates simultaneously
     * within a single 20 ms loop iteration. Without it, the robot arcs instead of
     * driving in a straight line.
     */
    public void drive(ChassisSpeeds speeds) {
        lastCommandedSpeeds = speeds;
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(speeds, 0.02)
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeedMetersPerSecond);

        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    /**
     * Drives using robot-relative ChassisSpeeds.
     * Used by movement commands that already handle the field→robot conversion.
     *
     * <p>Applies {@code ChassisSpeeds.discretize()} for the same skew correction
     * as {@link #drive(ChassisSpeeds)}.
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        lastCommandedSpeeds = speeds;
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(speeds, 0.02)
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeedMetersPerSecond);

        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    /** Stops all modules immediately. */
    public void stop() {
        lastCommandedSpeeds = new ChassisSpeeds();
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

    /** Returns the robot's current estimated field pose. */
    public Pose2d getPose() {
        return pose;
    }

    /** Returns the robot's current heading from the Pigeon2 (or simulated equivalent). */
    public Rotation2d getHeading() {
        if (RobotBase.isSimulation()) {
            return Rotation2d.fromRadians(simHeadingRadians);
        }
        return Rotation2d.fromDegrees(yaw.getValue().in(Degrees));
    }

    /** Returns the robot's current robot-relative chassis speeds derived from module states. */
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Returns the robot's velocity in field-relative coordinates.
     * Used by maintainHeadingToTarget for motion compensation.
     */
    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
            kinematics.toChassisSpeeds(getModuleStates()), getHeading()
        );
    }

    /** Returns the configured maximum translational speed in m/s. */
    public double getMaxSpeed() {
        return maxSpeedMetersPerSecond;
    }

    /** Returns the configured maximum angular rate in rad/s. */
    public double getMaxAngular() {
        return maxAngularMetersPerSecond;
    }

    // ============================================================
    // Vision & Localization
    // ============================================================

    /**
     * Feeds a vision-estimated pose into the pose estimator.
     *
     * @param visionPose        estimated robot pose from vision
     * @param timestampSeconds  FPGA timestamp of the measurement
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
     * Resets the robot's heading to 0° so its current facing direction becomes
     * field-forward. Also resets the pose estimator's heading component.
     */
    public void zeroHeading() {
        pigeon.setYaw(0.0);
        // In simulation, getHeading() reads simHeadingRadians directly — the Phoenix
        // sim backend doesn't update it synchronously from setYaw(). Mirror the reset
        // here so simulation and real hardware behave identically.
        simHeadingRadians = 0.0;
        Rotation2d zero = new Rotation2d();
        pose = new Pose2d(pose.getTranslation(), zero);
        poseEstimator.resetPosition(zero, getModulePositions(), pose);
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
            getHeading(),
            getModulePositions(),
            pose
        );
    }

    // ============================================================
    // Fault interpretation
    // ============================================================

    /** Returns true if any drive motor is currently reporting a stall condition. */
    public boolean isAnyDriveStalled() {
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) return false;

        return frontLeft.isDriveStalled()
            || frontRight.isDriveStalled()
            || backLeft.isDriveStalled()
            || backRight.isDriveStalled();
    }

    /** Returns true if any steer motor is currently reporting a stall condition. */
    public boolean isAnySteerStalled() {
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) return false;

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
        return MathUtil.clamp(error * HEADING_KP, -maxAngularMetersPerSecond, maxAngularMetersPerSecond);
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

    private SwerveModuleState[] getModuleStates() {
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
    // ║                        COMMANDS                                  ║
    // ║                                                                  ║
    // ║  All swerve commands live here, directly on the subsystem.       ║
    // ║  They use run() / runOnce() / andThen() from SubsystemBase,      ║
    // ║  which automatically registers this subsystem as a requirement.  ║
    // ║                                                                  ║
    // ║  SECTIONS:                                                       ║
    // ║    1. Basic Drive (teleop)                                       ║
    // ║    2. Heading Control (pointing, aiming)                         ║
    // ║    3. Autonomous Movement (driveToPoint, moveThroughPose, etc.)  ║
    // ║    4. Utility (brake, resetPose)                                 ║
    // ╚══════════════════════════════════════════════════════════════════╝


    // ─────────────────────────────────────────────────────────────────
    // 1. BASIC DRIVE
    // ─────────────────────────────────────────────────────────────────

    /**
     * Default teleop drive command.
     * Accepts a ChassisSpeeds supplier so Robot.java can compute speeds
     * from controller inputs each loop iteration.
     *
     *   swerve.setDefaultCommand(swerve.driveCommand(() -> computeSpeeds()));
     */
    public Command driveCommand(Supplier<ChassisSpeeds> speeds) {
        return run(() -> drive(speeds.get()))
            .finallyDo(interrupted -> stop())
            .withName("Swerve.drive");
    }

    /**
     * Drives at fixed ChassisSpeeds. Used by auto commands.
     */
    public Command driveCommand(ChassisSpeeds speeds) {
        return run(() -> drive(speeds))
            .finallyDo(interrupted -> stop())
            .withName("Swerve.driveFixed");
    }

    /**
     * Drives at the given ChassisSpeeds for a fixed number of seconds, then stops.
     * Called by AutoBuilder.driveForTime().
     */
    public Command driveForTime(ChassisSpeeds speeds, double seconds) {
        return run(() -> drive(speeds))
            .withTimeout(seconds)
            .finallyDo(interrupted -> stop())
            .withName("Swerve.driveForTime");
    }

    /** Stops all modules. Instantaneous (runOnce). */
    public Command stopCommand() {
        return runOnce(this::stop)
            .withName("Swerve.stop");
    }

    /**
     * Zeroes the gyro so the robot's current facing direction becomes field-forward (0°).
     * Bind to a controller button (left bumper is standard) and press while the robot
     * is physically pointing toward the opposing alliance wall.
     */
    public Command zeroHeadingCommand() {
        return runOnce(this::zeroHeading)
            .withName("Swerve.zeroHeading");
    }

    /** Resets the robot's pose estimate to a given field position. */
    public Command resetPoseCommand(Pose2d pose) {
        return runOnce(() -> resetPose(pose))
            .withName("Swerve.resetPose");
    }


    // ─────────────────────────────────────────────────────────────────
    // 2. HEADING CONTROL
    // ─────────────────────────────────────────────────────────────────

    /**
     * Rotates the robot in place until it faces a fixed field position.
     *
     * The robot looks toward the XY of targetPose — the target's own rotation
     * is ignored. Finishes within HEADING_TOLERANCE_DEG (5°).
     *
     * Use case: snap to face the speaker/hub before shooting.
     *
     * @param targetPose  the field position to face toward
     */
    public Command pointToTarget(Pose2d targetPose) {
        return run(() -> {
            Pose2d current     = getPose();
            Rotation2d desired = computeHeadingToTarget(current, targetPose);
            drive(new ChassisSpeeds(0.0, 0.0,
                computeOmega(current.getRotation(), desired)));
        })
        .until(() -> {
            Pose2d current     = getPose();
            Rotation2d desired = computeHeadingToTarget(current, targetPose);
            return Math.abs(MathUtil.angleModulus(
                current.getRotation().minus(desired).getRadians()
            )) < Math.toRadians(HEADING_TOLERANCE_DEG);
        })
        .finallyDo(interrupted -> stop())
        .withName("Swerve.pointToTarget");
    }

    /**
     * Rotates the robot in place to continuously face a dynamic target.
     * The target pose is re-read from the supplier every loop.
     *
     * Use case: continuously track a moving vision target or live shooter setpoint.
     *
     * @param targetPoseSupplier  supplier that returns the current target pose
     */
    public Command pointToTarget(Supplier<Pose2d> targetPoseSupplier) {
        return run(() -> {
            Pose2d current     = getPose();
            Rotation2d desired = computeHeadingToTarget(current, targetPoseSupplier.get());
            drive(new ChassisSpeeds(0.0, 0.0,
                computeOmega(current.getRotation(), desired)));
        })
        .until(() -> {
            Pose2d current     = getPose();
            Rotation2d desired = computeHeadingToTarget(current, targetPoseSupplier.get());
            return Math.abs(MathUtil.angleModulus(
                current.getRotation().minus(desired).getRadians()
            )) < Math.toRadians(HEADING_TOLERANCE_DEG);
        })
        .finallyDo(interrupted -> stop())
        .withName("Swerve.pointToTargetDynamic");
    }

    /**
     * Rotates the robot in place to match a specific heading in degrees.
     *
     * Finishes within HEADING_TOLERANCE_DEG (5°).
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
     * Finishes within HEADING_TOLERANCE_DEG (5°).
     *
     * @param targetHeading  the desired heading
     */
    public Command changeHeading(Rotation2d targetHeading) {
        return run(() -> {
            double omega = computeOmega(getPose().getRotation(), targetHeading);
            drive(new ChassisSpeeds(0.0, 0.0, omega));
        })
        .until(() ->
            Math.abs(MathUtil.angleModulus(
                getPose().getRotation().minus(targetHeading).getRadians()
            )) < Math.toRadians(HEADING_TOLERANCE_DEG)
        )
        .finallyDo(interrupted -> stop())
        .withName("Swerve.changeHeading");
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
     * arrive correctly even while moving. Tune AIM_LEAD_TIME at the top of this file.
     *
     * This command NEVER finishes on its own. Bind with whileTrue() so it cancels
     * when the button is released, returning control to the default drive command.
     *
     * Use case: driver holds a button to auto-aim while keeping full translation control.
     *
     * @param targetPoseSupplier  field target to aim at (re-read every loop)
     * @param xSupplier           driver forward/back input (m/s, field-relative)
     * @param ySupplier           driver strafe input (m/s, field-relative)
     */
    public Command maintainHeadingToTarget(
        Supplier<Pose2d> targetPoseSupplier,
        Supplier<Double> xSupplier,
        Supplier<Double> ySupplier
    ) {
        return run(() -> {
            Pose2d currentPose  = getPose();
            Pose2d targetPose   = targetPoseSupplier.get();
            ChassisSpeeds field = getFieldRelativeSpeeds();

            // Shift the aim point forward in time to compensate for robot motion.
            // If the robot moves right at 2 m/s and lead time is 0.25 s,
            // the compensated target shifts 0.5 m right — correcting for
            // the time the game piece takes to travel to the target.
            Translation2d compensated = targetPose.getTranslation().minus(
                new Translation2d(
                    field.vxMetersPerSecond * AIM_LEAD_TIME,
                    field.vyMetersPerSecond * AIM_LEAD_TIME
                )
            );

            Rotation2d desiredHeading = compensated
                .minus(currentPose.getTranslation())
                .getAngle();

            double omega = computeOmega(currentPose.getRotation(), desiredHeading);
            // xSupplier / ySupplier are field-relative — convert to robot-relative
            // before passing to drive() so translation is correct at any heading.
            drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                xSupplier.get(), ySupplier.get(), omega, currentPose.getRotation()
            ));
        })
        .finallyDo(interrupted -> stop())
        .withName("Swerve.maintainHeadingToTarget");
    }


    // ─────────────────────────────────────────────────────────────────
    // 3. AUTONOMOUS MOVEMENT
    // ─────────────────────────────────────────────────────────────────

    /**
     * Drives toward a target pose at a fixed velocity and stops when within
     * ARRIVE_THRESHOLD (5 cm) of the target XY.
     *
     * Rotation toward the target pose's heading is corrected proportionally
     * throughout the move.
     *
     * @param targetPose  the field pose to drive to
     * @param velocity    translation speed (m/s)
     * @param stopAtEnd   true = stop when done; false = leave velocity applied (for chaining)
     */
    public Command driveToPoint(Pose2d targetPose, double velocity, boolean stopAtEnd) {
        // stall[0,1] = last XY where movement was detected; stall[2] = timestamp
        final double[] stall = new double[3];

        return runOnce(() -> {
            Pose2d p = getPose();
            stall[0] = p.getX();
            stall[1] = p.getY();
            stall[2] = Timer.getFPGATimestamp();
        })
        .andThen(run(() -> {
            Pose2d current     = getPose();
            Rotation2d heading = current.getRotation();

            double dx       = targetPose.getX() - current.getX();
            double dy       = targetPose.getY() - current.getY();
            double distance = Math.hypot(dx, dy);

            double apfSpeed = Math.min(ARRIVE_KP * distance, velocity);
            double ux = (distance > ARRIVE_THRESHOLD) ? dx / distance : 0.0;
            double uy = (distance > ARRIVE_THRESHOLD) ? dy / distance : 0.0;

            driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(
                ux * apfSpeed, uy * apfSpeed,
                computeOmega(heading, targetPose.getRotation()),
                heading
            ));

            // Advance stall checkpoint whenever the robot makes meaningful progress
            double moveDx = Math.abs(current.getX() - stall[0]);
            double moveDy = Math.abs(current.getY() - stall[1]);
            if (moveDx > STALL_THRESHOLD || moveDy > STALL_THRESHOLD) {
                stall[0] = current.getX();
                stall[1] = current.getY();
                stall[2] = Timer.getFPGATimestamp();
            }
        }))
        .until(() -> {
            Pose2d current = getPose();
            if (current.getTranslation().getDistance(targetPose.getTranslation()) < ARRIVE_THRESHOLD)
                return true;
            double dx = Math.abs(current.getX() - stall[0]);
            double dy = Math.abs(current.getY() - stall[1]);
            return dx < STALL_THRESHOLD
                && dy < STALL_THRESHOLD
                && (Timer.getFPGATimestamp() - stall[2]) > STALL_TIMEOUT;
        })
        .finallyDo(interrupted -> { if (stopAtEnd) stop(); })
        .withName("Swerve.driveToPoint");
    }

    /**
     * Drives through a waypoint at constant speed without slowing down.
     *
     * Unlike driveToPoint, the command finishes as soon as the robot enters the
     * pass radius — there is no deceleration. This allows smooth, high-speed
     * paths through multiple chained waypoints.
     *
     * Rotation is controlled by a PID toward the target pose's heading.
     * Tune THETA_KP / KI / KD and MOVE_THROUGH_DEFAULT_RADIUS at the top of this file.
     *
     * Stall detection prevents the robot from getting permanently stuck if
     * it hits an obstacle — the command auto-exits after STALL_TIMEOUT seconds
     * of insufficient movement.
     *
     * @param targetPose  the waypoint pose to pass through
     * @param maxSpeed    translation speed (m/s)
     * @param maxAngular  maximum rotation speed (rad/s), clamps PID output
     * @param passRadius  distance to consider the waypoint "passed" (meters)
     */
    @SuppressWarnings("resource")
    public Command moveThroughPose(
        Pose2d targetPose,
        double maxSpeed,
        double maxAngular,
        double passRadius
    ) {
        PIDController thetaPID = new PIDController(THETA_KP, THETA_KI, THETA_KD);
        thetaPID.enableContinuousInput(-Math.PI, Math.PI);

        // stall[0,1] = last XY where movement was detected; stall[2] = stall timestamp; stall[3] = command start
        final double[] stall = new double[4];

        return runOnce(() -> {
            Pose2d p = getPose();
            stall[0] = p.getX();
            stall[1] = p.getY();
            stall[2] = Timer.getFPGATimestamp();
            stall[3] = stall[2]; // wall-clock start for hard deadline
            thetaPID.reset();
        })
        .andThen(run(() -> {
            Pose2d current = getPose();

            // Normalized direction vector toward waypoint
            double dx       = targetPose.getX() - current.getX();
            double dy       = targetPose.getY() - current.getY();
            double distance = Math.hypot(dx, dy);
            double dirX     = dx / (distance + 1e-9);
            double dirY     = dy / (distance + 1e-9);

            // PID rotation toward target heading, clamped to maxAngular
            double omega = MathUtil.clamp(
                thetaPID.calculate(
                    current.getRotation().getRadians(),
                    targetPose.getRotation().getRadians()
                ),
                -maxAngular, maxAngular
            );

            driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(
                dirX * maxSpeed, dirY * maxSpeed, omega, current.getRotation()
            ));

            // Update stall tracker if the robot is actually moving
            double moveDx = Math.abs(current.getX() - stall[0]);
            double moveDy = Math.abs(current.getY() - stall[1]);
            if (moveDx > STALL_THRESHOLD || moveDy > STALL_THRESHOLD) {
                stall[0] = current.getX();
                stall[1] = current.getY();
                stall[2] = Timer.getFPGATimestamp();
            }
        }))
        .until(() -> {
            Pose2d current = getPose();

            // Done: robot entered pass radius
            if (current.getTranslation().getDistance(targetPose.getTranslation()) < passRadius)
                return true;

            // Done: stall timeout expired (robot not moving)
            double dx = Math.abs(current.getX() - stall[0]);
            double dy = Math.abs(current.getY() - stall[1]);
            if (dx < STALL_THRESHOLD && dy < STALL_THRESHOLD
                    && (Timer.getFPGATimestamp() - stall[2]) > STALL_TIMEOUT)
                return true;

            // Done: hard wall-clock deadline — catches oscillation that keeps resetting the stall timer
            return (Timer.getFPGATimestamp() - stall[3]) > MAX_MOVETHROUGH_SECONDS;
        })
        .finallyDo(interrupted -> stop())
        .withName("Swerve.moveThroughPose");
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
     * The target pose is computed from the robot's starting position and heading
     * at the moment the command initializes. Heading is held with proportional
     * correction throughout. Finishes within ARRIVE_THRESHOLD (5 cm).
     *
     * Use case: fallback auto when vision is offline — just drive a known distance.
     *
     * @param distanceMeters  distance to drive (meters, positive = forward)
     * @param velocity        translation speed (m/s)
     * @param stopAtEnd       true = stop when done
     */
    public Command driveForwardMeters(double distanceMeters, double velocity, boolean stopAtEnd) {
        // Single-element array so the target pose computed in runOnce is accessible
        // in the run() and until() closures without subsystem-level mutable state.
        final Pose2d[] target = { new Pose2d() };

        return runOnce(() -> {
            Pose2d current = getPose();
            Rotation2d hdg = current.getRotation();
            target[0] = new Pose2d(
                current.getX() + distanceMeters * hdg.getCos(),
                current.getY() + distanceMeters * hdg.getSin(),
                hdg
            );
        })
        .andThen(run(() -> {
            Pose2d current = getPose();
            Rotation2d hdg = current.getRotation();

            double dx       = target[0].getX() - current.getX();
            double dy       = target[0].getY() - current.getY();
            double distance = Math.hypot(dx, dy);

            double apfSpeed = Math.min(ARRIVE_KP * distance, velocity);
            double ux = (distance > ARRIVE_THRESHOLD) ? dx / distance : 0.0;
            double uy = (distance > ARRIVE_THRESHOLD) ? dy / distance : 0.0;

            driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(
                ux * apfSpeed, uy * apfSpeed,
                computeOmega(hdg, target[0].getRotation()),
                hdg
            ));
        }))
        .until(() ->
            getPose().getTranslation().getDistance(target[0].getTranslation()) < ARRIVE_THRESHOLD
        )
        .finallyDo(interrupted -> { if (stopAtEnd) stop(); })
        .withName("Swerve.driveForwardMeters");
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
        return run(this::brake)
            .finallyDo(interrupted -> stop())
            .withName("Swerve.brake");
    }
}
