//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.swerve;

import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.framework.RobotBase;

import org.team1507.lib.core.impl.ctre.Motor1507;
import org.team1507.lib.core.logging.Telemetry;

public final class SwerveModule1507 {

    public record MathConfig(
        double driveGearRatio,
        double steerGearRatio,
        double couplingRatio,
        double wheelRadiusMeters
    ) {
        public double wheelCircumferenceMeters() {
            return 2.0 * Math.PI * wheelRadiusMeters;
        }
    }

    private final String name;

    private final Motor1507 drive;
    private final Motor1507 steer;
    private final CANcoder encoder;

    private final MathConfig math;
    private final double driveMetersScale;

    private Rotation2d lastAngle = Rotation2d.ZERO;

    // Sim state
    private double simSteerAngleRotations = 0.0;
    private double simDriveVelocityRps = 0.0;
    private double simDrivePositionRotations = 0.0;

    /**
     * CANcoder position in module rotations. The magnet offset is stored on the
     * CANcoder (see Swerve.createModule), so 0 = wheel pointing forward.
     *
     * <p>This is the CONTINUOUS position (it counts past 1.0 rotation), not
     * getAbsolutePosition(), which wraps from +0.5 to -0.5. The coupling
     * correction below multiplies this value, so a wrap would add a fake jump
     * of ~18 cm to odometry every time a wheel crossed the wrap point.
     */
    private final StatusSignal<Angle> azimuthPosition;
    private final StatusSignal<AngularVelocity> azimuthVelocity;

    private final BaseStatusSignal[] allSignals;

    public SwerveModule1507(
        String name,
        Motor1507 drive,
        Motor1507 steer,
        CANcoder encoder,
        MathConfig math,
        double driveMetersScale
    ) {
        this.name = name;
        this.drive = drive;
        this.steer = steer;
        this.encoder = encoder;
        this.math = math;
        this.driveMetersScale = driveMetersScale;

        this.azimuthPosition = encoder.getPosition();
        this.azimuthVelocity = encoder.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            azimuthPosition,
            azimuthVelocity
        );

        // Every drive, steer and CANcoder signal, refreshed together by Swerve.
        // Built from the motors' own lists, so adding a motor signal can't break it.
        java.util.List<BaseStatusSignal> signals = new java.util.ArrayList<>();
        signals.addAll(java.util.List.of(drive.getSignals()));
        signals.addAll(java.util.List.of(steer.getSignals()));
        signals.add(azimuthPosition);
        signals.add(azimuthVelocity);
        this.allSignals = signals.toArray(BaseStatusSignal[]::new);

        Telemetry.set(key("Drive/MetersScale"), driveMetersScale);
        Telemetry.set(key("Initialized"), true);
    }

    // ============================================================
    // Control
    // ============================================================

    /**
     * Applies a desired swerve module state (speed and angle), optimizing direction
     * to minimize wheel rotation and applying an anti-jitter guard at near-zero speed.
     *
     * @param desired the target module state
     */
    public void setDesiredState(SwerveModuleVelocity desired) {
        Rotation2d current = getAngle();
        SwerveModuleVelocity optimized = new SwerveModuleVelocity(
            desired.velocity,
            desired.angle
        );

        // 2027: SwerveModuleVelocity is immutable. optimize() returns a NEW object
        // instead of changing this one, so the result must be assigned back.
        optimized = optimized.optimize(current);

        Rotation2d targetAngle =
            Math.abs(optimized.velocity) < 0.01
                ? lastAngle
                : optimized.angle;

        double driveRps =
            metersPerSecondToDriveMotorRps(optimized.velocity);

        drive.setRPS(driveRps);
        steer.setPositionRotations(targetAngle.getRotations());

        // In sim, instantly snap to the target so getAngle() returns
        // the correct value next loop — simulates a perfect steer controller
        if (RobotBase.isSimulation()) {
            simSteerAngleRotations = targetAngle.getRotations();
            simDriveVelocityRps = driveRps / math.driveGearRatio();
        }

        lastAngle = targetAngle;

        Telemetry.set(key("Drive/TargetMps"), optimized.velocity);
        Telemetry.set(key("Drive/TargetMotorRps"), driveRps);
        Telemetry.set(key("Steer/TargetAngleRad"), targetAngle.getRadians());
    }

    /** Stops both the drive and steer motors immediately. */
    public void stop() {
        drive.stop();
        steer.stop();

        if (RobotBase.isSimulation()) {
            simDriveVelocityRps = 0.0;
        }
    }

    /**
     * Commands the steer directly to a target angle and stops the drive motor.
     *
     * <p>Unlike {@link #setDesiredState}, this method bypasses the anti-jitter
     * guard that skips steer updates when speed is near zero. Use this for
     * X-brake mode where the target angle must be applied regardless
     * of drive speed.
     *
     * @param angle the target steer angle
     */
    public void brakeToAngle(Rotation2d angle) {
        drive.stop();
        steer.setPositionRotations(angle.getRotations());

        if (RobotBase.isSimulation()) {
            simSteerAngleRotations = angle.getRotations();
            simDriveVelocityRps    = 0.0;
        }

        lastAngle = angle;
    }

    // ============================================================
    // Simulation Update
    // ============================================================

    /**
     * Called every loop from Swerve.simulationPeriodic().
     * Updates the CANcoder sim state so getAngle() returns real values,
     * and integrates drive position.
     */
    public void simulationUpdate(double dtSeconds) {
        // Keep the CANcoder sim state in step with the simulated steer angle.
        // (getAngle() reads the sim value directly; this keeps the device's
        // own signals plausible for anything that reads them.)
        CANcoderSimState encoderSim = encoder.getSimState();
        encoderSim.setRawPosition(simSteerAngleRotations);
        encoderSim.setVelocity(0.0);

        // Integrate drive position
        simDrivePositionRotations += simDriveVelocityRps * dtSeconds;
    }

    // ============================================================
    // Observation
    // ============================================================

    /** Returns the current steer angle (0 = wheel forward; offset is applied on the CANcoder). */
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(azimuthRotations());
    }

    /** Continuous azimuth position in module rotations (does not wrap). */
    private double azimuthRotations() {
        if (RobotBase.isSimulation()) {
            return simSteerAngleRotations;
        }
        return azimuthPosition.getValue().in(Rotations);
    }

    private double azimuthRpsRaw() {
        return RobotBase.isSimulation() ? 0.0
            : azimuthVelocity.getValue().in(RotationsPerSecond);
    }

    /** Returns the current module state (speed in m/s and steer angle). */
    public SwerveModuleVelocity getState() {
        double mps = RobotBase.isSimulation()
            ? simDriveVelocityRps * math.wheelCircumferenceMeters()
            : wheelRpsToMetersPerSecond(correctedWheelRps()) * driveMetersScale;

        return new SwerveModuleVelocity(mps, getAngle());
    }

    /** Returns the current module position (distance traveled in meters and steer angle). */
    public SwerveModulePosition getPosition() {
        double meters = RobotBase.isSimulation()
            ? simDrivePositionRotations * math.wheelCircumferenceMeters()
            : wheelRotationsToMeters(correctedWheelRotations()) * driveMetersScale;

        return new SwerveModulePosition(meters, getAngle());
    }

    /** Returns all CAN status signals for this module (drive, steer, and encoder), for batched refresh. */
    public BaseStatusSignal[] getAllSignals() {
        return allSignals;
    }

    /** @deprecated Replaced by {@link #getAllSignals()} — Swerve now batches all signals in one call. */
    @Deprecated
    public void refreshSignals() {
        // no-op — Swerve.periodic() calls BaseStatusSignal.refreshAll(allSignals) for all modules
    }

    // ============================================================
    // CAN Bus Access (for bus optimization only)
    // ============================================================

    /** Returns the drive motor's Phoenix 6 device handle. Used to optimize CAN bus utilization. */
    public com.ctre.phoenix6.hardware.ParentDevice getDriveDevice() { return drive.getDevice(); }

    /** Returns the steer motor's Phoenix 6 device handle. Used to optimize CAN bus utilization. */
    public com.ctre.phoenix6.hardware.ParentDevice getSteerDevice() { return steer.getDevice(); }

    /** Returns the CANcoder's Phoenix 6 device handle. Used to optimize CAN bus utilization. */
    public com.ctre.phoenix6.hardware.ParentDevice getEncoderDevice() { return encoder; }

    // ============================================================
    // Faults
    // ============================================================

    /** Returns true if the drive motor is currently reporting a stall condition. */
    public boolean isDriveStalled() {
        return drive.isStalled();
    }

    /** Returns true if the steer motor is currently reporting a stall condition. */
    public boolean isSteerStalled() {
        return steer.isStalled();
    }

    // ============================================================
    // Core math
    // ============================================================

    private double correctedDriveMotorRotations() {
        return drive.getPositionRotations()
            - (azimuthRotations() * math.couplingRatio());
    }

    private double correctedDriveMotorRps() {
        return drive.getRPS()
            - (azimuthRpsRaw() * math.couplingRatio());
    }

    private double correctedWheelRotations() {
        return correctedDriveMotorRotations() / math.driveGearRatio();
    }

    private double correctedWheelRps() {
        return correctedDriveMotorRps() / math.driveGearRatio();
    }

    private double wheelRotationsToMeters(double wheelRotations) {
        return wheelRotations * math.wheelCircumferenceMeters();
    }

    private double wheelRpsToMetersPerSecond(double wheelRps) {
        return wheelRps * math.wheelCircumferenceMeters();
    }

    private double metersPerSecondToDriveMotorRps(double mps) {
        double wheelRps = mps / math.wheelCircumferenceMeters();
        return (wheelRps * math.driveGearRatio())
            + (azimuthRpsRaw() * math.couplingRatio());
    }

    private String key(String field) {
        return "Swerve/" + name + "/" + field;
    }
}
