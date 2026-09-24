//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.util;

import static org.wpilib.units.Units.Amps;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import org.wpilib.units.measure.Current;

/**
 * Declarative motor configuration for {@link org.team1507.lib.core.impl.ctre.Motor1507}.
 *
 * <p>Describes all hardware settings for a CTRE TalonFX or TalonFXS in one
 * immutable value, using simple names instead of CTRE's long config names.
 *
 * <h2>Start from a preset</h2>
 * A preset fills in everything that is the same for every mechanism of that
 * kind (control mode, brake/coast, current limits, gravity type, tolerance).
 * You add only what is specific to YOUR mechanism, and anything you add wins:
 *
 * <pre>
 *   public static final MotorConfig RIGHT_ARM = MotorConfig.arm()
 *       .gearRatio(50.0)            // motor turns per arm turn
 *       .withPID(0.5, 0.0, 0.0)
 *       .withGravity(0.1)
 *       .build();
 *
 *   // Same arm on the other side, mirrored:
 *   public static final MotorConfig LEFT_ARM = MotorConfig.copy(RIGHT_ARM)
 *       .inverted(true)
 *       .build();
 * </pre>
 *
 * <table>
 *   <caption>Preset starting values (all overridable)</caption>
 *   <tr><th>Preset</th><th>Control</th><th>Neutral</th><th>Stator / supply</th><th>Other</th></tr>
 *   <tr><td>{@link #roller()}</td><td>torque (Phoenix Pro)</td><td>coast</td><td>80 A / 35 A, 15 A after 4 s</td><td>stall detection</td></tr>
 *   <tr><td>{@link #flywheel()}</td><td>velocity</td><td>coast</td><td>80 A / 40 A</td><td>50 RPM tolerance</td></tr>
 *   <tr><td>{@link #arm()}</td><td>position</td><td>brake</td><td>60 A / 30 A</td><td>cosine gravity, 2° tolerance</td></tr>
 *   <tr><td>{@link #elevator()}</td><td>position</td><td>brake</td><td>80 A / 30 A</td><td>constant gravity, 1 cm tolerance, needs a drum diameter</td></tr>
 *   <tr><td>{@link #position()}</td><td>position</td><td>brake</td><td>60 A / 30 A</td><td>no gravity, 2° tolerance</td></tr>
 * </table>
 *
 * <p>Sources for the starting values: CTRE's current-limit guide (elevator
 * 80 A / 30 A), Team 340's 2026 robot (flywheel 80 A / 40 A, coast; rollers with
 * a low sustained supply limit). Supply limits are our main brownout
 * protection, so every preset has one.
 *
 * <h2>Anything CTRE has that this doesn't</h2>
 * {@link Builder#withCtreConfig(Consumer)} runs LAST, so you can paste a CTRE
 * example unchanged and it overrides everything above it.
 *
 * <h2>Checked on every build</h2>
 * {@link #problems()} lists mistakes that compile fine but break the robot.
 * {@code MotorConfigConstantsTest} runs it on every MotorConfig in Constants.java.
 *
 * <p>Multiple configs can be passed to one motor to fill several PID slots.
 * The first config (slot 0) also defines all base hardware settings.
 */
public record MotorConfig(
        Preset preset,
        int slotNumber,

        ControlMode mode,
        boolean motorInverted,

        double kP, double kI, double kD,
        double kV, double kS, double kA,
        StaticFeedforwardSignValue staticFeedforwardSign,

        double kG,
        GravityType gravityType,

        double peakForwardVoltage,
        double peakReverseVoltage,

        Current statorCurrentLimit,
        Current supplyCurrentLimit,
        /** Supply limit after {@link #supplyLowerTime()} seconds of limiting; NaN = not used. */
        double supplyLowerLimitAmps,
        double supplyLowerTime,

        double peakForwardTorqueCurrent,
        double peakReverseTorqueCurrent,

        boolean forwardLimitEnable,
        boolean forwardLimitAutosetEnable,
        double forwardLimitAutosetValue,
        ForwardLimitTypeValue forwardLimitType,

        boolean reverseLimitEnable,
        boolean reverseLimitAutosetEnable,
        double reverseLimitAutosetValue,
        ReverseLimitTypeValue reverseLimitType,

        Feedback feedback,

        /** Drum/sprocket circumference for linear mechanisms (meters); NaN = rotary. */
        double drumCircumferenceMeters,

        /** "At target" tolerances used by Motor1507.isAtTarget(); NaN = not set. */
        double toleranceDegrees,
        double toleranceMeters,
        double toleranceRpm,

        // ---------------------------
        // Stall detection thresholds
        // ---------------------------
        double stallCurrentThreshold,
        double stallVelocityThreshold,
        double stallTimeSeconds,

        boolean brakeMode,
        boolean continuousWrap,
        boolean enableFOC,

        double simVelocityRps,

        /** Raw CTRE overrides, applied last. Null = none. */
        Consumer<TalonFXConfiguration> ctreFxConfig,
        Consumer<TalonFXSConfiguration> ctreFxsConfig
) {

    /** Which preset a config started from (CUSTOM = plain builder). */
    public enum Preset { CUSTOM, ROLLER, FLYWHEEL, ARM, ELEVATOR, POSITION }

    /**
     * Selects the control strategy applied by the configurator and motor.
     *
     * <p>{@code DUTY_CYCLE} and {@code TORQUE} are open-loop — PID and feedforward
     * slot gains are not written to hardware for these modes.
     */
    public enum ControlMode { DUTY_CYCLE, TORQUE, VELOCITY, POSITION, MOTION_MAGIC }

    public enum GravityType {
        /** No gravity compensation. */
        NONE,
        /** Arm mechanisms — kG × cos(position). Maps to {@code Arm_Cosine}. */
        COSINE,
        /** Elevator mechanisms — constant kG offset. Maps to {@code Elevator_Static}. */
        CONSTANT
    }

    /**
     * Feedback sensor configuration applied to the motor's closed-loop controller.
     *
     * @param source                 sensor source (rotor, CANcoder, remote, etc.)
     * @param remoteSensorId         CAN ID of the remote sensor, if applicable
     * @param rotorToSensorRatio     gear ratio from motor rotor to the sensor shaft
     * @param sensorToMechanismRatio gear ratio from the sensor shaft to the mechanism output
     * @param rotorOffset            rotor position offset applied at startup (rotations)
     */
    public record Feedback(
        FeedbackSensorSourceValue source,
        int remoteSensorId,
        double rotorToSensorRatio,
        double sensorToMechanismRatio,
        double rotorOffset
    ) {}

    private static final double INCHES_TO_METERS = 0.0254;

    // =========================================================================
    // Starting points
    // =========================================================================

    /** A plain builder with CTRE-like defaults and {@code DUTY_CYCLE} mode. Prefer a preset. */
    public static Builder builder() {
        return new Builder();
    }

    /** A plain builder with the given control mode. Prefer a preset. */
    public static Builder builder(ControlMode mode) {
        return new Builder().mode(mode);
    }

    /**
     * Intake/feeder rollers where FORCE matters more than speed: torque control
     * keeps pushing as the load grows (e.g. more game pieces in the hopper).
     * Coast, 80 A stator / 35 A supply dropping to 15 A after 4 s of limiting,
     * so a jammed roller can't drain the battery. Stall detection on.
     *
     * <p>Torque control (TorqueCurrentFOC) requires a Phoenix Pro license. For a
     * roller that should hold a SPEED instead, use {@link #flywheel()}.
     */
    public static Builder roller() {
        return new Builder()
            .preset(Preset.ROLLER)
            .mode(ControlMode.TORQUE)
            .withCoast()
            .withStatorCurrentLimit(80.0)
            .withSupplyCurrentLimit(35.0)
            .withSupplyLowerLimit(15.0, 4.0)
            .withPeakTorqueCurrent(80.0, -80.0)
            .withFOC();
    }

    /**
     * Shooter flywheels and anything that should hold a SPEED.
     * Velocity control, coast, 80 A stator / 40 A supply, 50 RPM tolerance.
     * Tuning tip (CTRE): kP about 0.1–0.4, kD = 0, and never use ramp rates.
     */
    public static Builder flywheel() {
        return new Builder()
            .preset(Preset.FLYWHEEL)
            .mode(ControlMode.VELOCITY)
            .withCoast()
            .withStatorCurrentLimit(80.0)
            .withSupplyCurrentLimit(40.0)
            .withToleranceRPM(50.0);
    }

    /**
     * Arms and pivots that gravity pulls down harder when horizontal.
     * Position control, brake, cosine gravity, 60 A stator / 30 A supply,
     * 2° tolerance. Set {@code gearRatio()} so positions are in arm degrees,
     * and zero the arm when horizontal so cosine gravity is correct.
     */
    public static Builder arm() {
        return new Builder()
            .preset(Preset.ARM)
            .mode(ControlMode.POSITION)
            .withBrake()
            .gravityType(GravityType.COSINE)
            .withStatorCurrentLimit(60.0)
            .withSupplyCurrentLimit(30.0)
            .withToleranceDegrees(2.0);
    }

    /**
     * Elevators and other straight-line mechanisms gravity pulls on evenly.
     * Position control, brake, constant gravity, 80 A stator / 30 A supply,
     * 1 cm tolerance. REQUIRES a drum/sprocket diameter so the motor can work
     * in meters/inches: {@code .withDrumDiameterMeters(0.05)}.
     */
    public static Builder elevator() {
        return new Builder()
            .preset(Preset.ELEVATOR)
            .mode(ControlMode.POSITION)
            .withBrake()
            .gravityType(GravityType.CONSTANT)
            .withStatorCurrentLimit(80.0)
            .withSupplyCurrentLimit(30.0)
            .withToleranceMeters(0.01);
    }

    /**
     * Anything that moves to an angle without fighting gravity: turrets, hoods,
     * hoppers. Position control, brake, 60 A stator / 30 A supply, 2° tolerance.
     */
    public static Builder position() {
        return new Builder()
            .preset(Preset.POSITION)
            .mode(ControlMode.POSITION)
            .withBrake()
            .withStatorCurrentLimit(60.0)
            .withSupplyCurrentLimit(30.0)
            .withToleranceDegrees(2.0);
    }

    /**
     * A builder pre-filled with every value from {@code original}. Change what
     * differs, then build:
     *
     * <pre>
     *   LEFT_SHOOTER = MotorConfig.copy(RIGHT_SHOOTER).inverted(true).build();
     * </pre>
     */
    public static Builder copy(MotorConfig original) {
        return new Builder(original);
    }

    // =========================================================================
    // Validation
    // =========================================================================

    /**
     * Mistakes that compile fine but break the robot, one plain-English message
     * each. Empty means the config looks sane. Checked on every build by
     * {@code MotorConfigConstantsTest} for every MotorConfig in Constants.java.
     */
    public List<String> problems() {
        List<String> problems = new ArrayList<>();

        if (slotNumber < 0 || slotNumber > 2) {
            problems.add("slot must be 0, 1 or 2 (got " + slotNumber + ").");
        }
        if (statorCurrentLimit.in(Amps) <= 0) {
            problems.add("stator current limit must be positive.");
        }
        if (supplyCurrentLimit.in(Amps) <= 0) {
            problems.add("supply current limit must be positive: it is the brownout protection.");
        }
        if (!Double.isNaN(supplyLowerLimitAmps)) {
            if (supplyLowerLimitAmps <= 0 || supplyLowerLimitAmps > supplyCurrentLimit.in(Amps)) {
                problems.add("supply lower limit (" + supplyLowerLimitAmps
                    + " A) must be positive and no higher than the supply limit ("
                    + supplyCurrentLimit.in(Amps) + " A).");
            }
            if (supplyLowerTime <= 0) {
                problems.add("supply lower limit time must be positive.");
            }
        }
        if (peakForwardVoltage <= 0 || peakReverseVoltage >= 0) {
            problems.add("voltage limits must be forward > 0 and reverse < 0.");
        }
        if (peakForwardTorqueCurrent < 0 || peakReverseTorqueCurrent > 0) {
            problems.add("peak torque current must be forward >= 0 and reverse <= 0.");
        }
        if (feedback.sensorToMechanismRatio() <= 0 || feedback.rotorToSensorRatio() <= 0) {
            problems.add("gear ratios must be positive (motor turns per mechanism turn).");
        }
        if (!Double.isNaN(drumCircumferenceMeters) && drumCircumferenceMeters <= 0) {
            problems.add("drum diameter must be positive.");
        }
        if (preset == Preset.ELEVATOR && Double.isNaN(drumCircumferenceMeters)) {
            problems.add("elevator() needs a drum/sprocket diameter: "
                + ".withDrumDiameterMeters(...) or .withDrumDiameterInches(...).");
        }
        if ((mode == ControlMode.DUTY_CYCLE || mode == ControlMode.TORQUE)
                && (kP != 0 || kI != 0 || kD != 0 || kV != 0 || kS != 0 || kA != 0)) {
            problems.add(mode + " is open-loop, so PID/feedforward gains are ignored. "
                + "Remove them or use a closed-loop preset.");
        }
        if (mode == ControlMode.TORQUE && !enableFOC) {
            problems.add("TORQUE control needs FOC (Phoenix Pro): add .withFOC().");
        }
        if (gravityType != GravityType.NONE && mode != ControlMode.POSITION
                && mode != ControlMode.MOTION_MAGIC) {
            problems.add("gravity compensation only works with position control.");
        }
        return problems;
    }

    // =========================================================================
    // Builder
    // =========================================================================

    /**
     * Fluent builder for {@link MotorConfig}.
     *
     * <p>Start from a preset ({@link #arm()}, {@link #flywheel()}, ...) or
     * {@link #builder()}. Every method overrides whatever the preset set.
     */
    public static final class Builder {

        private Preset preset = Preset.CUSTOM;
        private int slotNumber = 0;

        private ControlMode mode = ControlMode.DUTY_CYCLE;
        private boolean motorInverted = false;

        private double kP = 0, kI = 0, kD = 0;
        private double kV = 0, kS = 0, kA = 0;
        private StaticFeedforwardSignValue staticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        private double kG = 0;
        private GravityType gravityType = GravityType.NONE;

        private double peakForwardVoltage = 12;
        private double peakReverseVoltage = -12;

        private Current statorCurrentLimit = Amps.of(120.0);
        private Current supplyCurrentLimit = Amps.of(70.0);
        private double supplyLowerLimitAmps = Double.NaN;
        private double supplyLowerTime = 1.0;

        // CTRE defaults — 800 A is effectively unlimited.
        // Only matters when using TorqueCurrentFOC (ControlMode.TORQUE).
        private double peakForwardTorqueCurrent =  800.0;
        private double peakReverseTorqueCurrent = -800.0;

        private boolean forwardLimitEnable = false;
        private boolean forwardLimitAutosetEnable = false;
        private double forwardLimitAutosetValue = 0.0;
        private ForwardLimitTypeValue forwardLimitType = ForwardLimitTypeValue.NormallyOpen;

        private boolean reverseLimitEnable = false;
        private boolean reverseLimitAutosetEnable = false;
        private double reverseLimitAutosetValue = 0.0;
        private ReverseLimitTypeValue reverseLimitType = ReverseLimitTypeValue.NormallyOpen;

        private FeedbackSensorSourceValue feedbackSource = FeedbackSensorSourceValue.RotorSensor;
        private int feedbackRemoteId = 0;
        private double rotorToSensorRatio = 1.0;
        private double sensorToMechanismRatio = 1.0;
        private double rotorOffset = 0.0;

        private double drumCircumferenceMeters = Double.NaN;

        private double toleranceDegrees = Double.NaN;
        private double toleranceMeters = Double.NaN;
        private double toleranceRpm = Double.NaN;

        private double stallCurrentThreshold = 60.0;   // amps
        private double stallVelocityThreshold = 0.2;   // rotations/sec
        private double stallTimeSeconds = 0.25;        // seconds

        private boolean brakeMode = false;
        private boolean continuousWrap = false;
        private boolean enableFOC = false;

        private double simVelocityRps = 0.0;

        private Consumer<TalonFXConfiguration> ctreFxConfig = null;
        private Consumer<TalonFXSConfiguration> ctreFxsConfig = null;

        private Builder() {}

        /** Copies every field of an existing config (used by {@link MotorConfig#copy}). */
        private Builder(MotorConfig c) {
            preset = c.preset;
            slotNumber = c.slotNumber;
            mode = c.mode;
            motorInverted = c.motorInverted;
            kP = c.kP; kI = c.kI; kD = c.kD;
            kV = c.kV; kS = c.kS; kA = c.kA;
            staticFeedforwardSign = c.staticFeedforwardSign;
            kG = c.kG;
            gravityType = c.gravityType;
            peakForwardVoltage = c.peakForwardVoltage;
            peakReverseVoltage = c.peakReverseVoltage;
            statorCurrentLimit = c.statorCurrentLimit;
            supplyCurrentLimit = c.supplyCurrentLimit;
            supplyLowerLimitAmps = c.supplyLowerLimitAmps;
            supplyLowerTime = c.supplyLowerTime;
            peakForwardTorqueCurrent = c.peakForwardTorqueCurrent;
            peakReverseTorqueCurrent = c.peakReverseTorqueCurrent;
            forwardLimitEnable = c.forwardLimitEnable;
            forwardLimitAutosetEnable = c.forwardLimitAutosetEnable;
            forwardLimitAutosetValue = c.forwardLimitAutosetValue;
            forwardLimitType = c.forwardLimitType;
            reverseLimitEnable = c.reverseLimitEnable;
            reverseLimitAutosetEnable = c.reverseLimitAutosetEnable;
            reverseLimitAutosetValue = c.reverseLimitAutosetValue;
            reverseLimitType = c.reverseLimitType;
            feedbackSource = c.feedback.source();
            feedbackRemoteId = c.feedback.remoteSensorId();
            rotorToSensorRatio = c.feedback.rotorToSensorRatio();
            sensorToMechanismRatio = c.feedback.sensorToMechanismRatio();
            rotorOffset = c.feedback.rotorOffset();
            drumCircumferenceMeters = c.drumCircumferenceMeters;
            toleranceDegrees = c.toleranceDegrees;
            toleranceMeters = c.toleranceMeters;
            toleranceRpm = c.toleranceRpm;
            stallCurrentThreshold = c.stallCurrentThreshold;
            stallVelocityThreshold = c.stallVelocityThreshold;
            stallTimeSeconds = c.stallTimeSeconds;
            brakeMode = c.brakeMode;
            continuousWrap = c.continuousWrap;
            enableFOC = c.enableFOC;
            simVelocityRps = c.simVelocityRps;
            ctreFxConfig = c.ctreFxConfig;
            ctreFxsConfig = c.ctreFxsConfig;
        }

        private Builder preset(Preset p) { this.preset = p; return this; }

        private Builder gravityType(GravityType type) { this.gravityType = type; return this; }

        // ── Basics ──────────────────────────────────────────────────────────

        /** Sets the control strategy. Presets already choose one. */
        public Builder mode(ControlMode m) { this.mode = m; return this; }

        /** Reverses the motor's positive direction. */
        public Builder inverted(boolean inv) { this.motorInverted = inv; return this; }

        /**
         * PID slot this config fills (0, 1 or 2). Pass several configs to one
         * motor to use several slots; slot 0 also sets all hardware settings.
         */
        public Builder slot(int slot) { this.slotNumber = slot; return this; }

        /** Brake when stopped: the motor actively resists motion. */
        public Builder withBrake() { this.brakeMode = true; return this; }

        /** Coast when stopped: the motor spins freely. */
        public Builder withCoast() { this.brakeMode = false; return this; }

        /**
         * FOC commutation: about 15% more power and more torque per amp.
         * <b>Requires a Phoenix Pro license on the motor.</b>
         */
        public Builder withFOC() { this.enableFOC = true; return this; }

        // ── Gains ───────────────────────────────────────────────────────────

        /** Closed-loop PID gains (volts per unit of error). */
        public Builder withPID(double p, double i, double d) {
            this.kP = p; this.kI = i; this.kD = d; return this;
        }

        /** Feedforward gains: kS (volts to start moving), kV (volts per rps), kA (volts per rps²). */
        public Builder withFeedforward(double kS, double kV, double kA) {
            this.kS = kS; this.kV = kV; this.kA = kA; return this;
        }

        /**
         * Chooses which direction kS pushes. The CTRE default,
         * {@code UseVelocitySign}, pushes in the direction of travel.
         * {@code UseClosedLoopSign} pushes toward the target, which stops
         * position loops (like swerve steering) from jittering at the setpoint
         * when kS is not zero. Tuner X's swerve generator uses UseClosedLoopSign
         * for steering.
         */
        public Builder withStaticFeedforwardSign(StaticFeedforwardSignValue sign) {
            this.staticFeedforwardSign = sign; return this;
        }

        /**
         * Gravity feedforward (volts to hold the mechanism against gravity).
         * The arm and elevator presets already know the gravity type.
         *
         * @throws IllegalStateException if no gravity type is known yet
         *     (use {@link #withGravity(double, GravityType)} instead)
         */
        public Builder withGravity(double kG) {
            if (gravityType == GravityType.NONE) {
                throw new IllegalStateException(
                    "withGravity(kG) needs a gravity type: use arm()/elevator(), "
                    + "or withGravity(kG, GravityType.COSINE or CONSTANT).");
            }
            this.kG = kG; return this;
        }

        /** Gravity feedforward with an explicit type (COSINE for arms, CONSTANT for elevators). */
        public Builder withGravity(double kG, GravityType type) {
            this.kG = kG; this.gravityType = type; return this;
        }

        // ── Mechanism geometry ───────────────────────────────────────────────

        /**
         * Motor turns per mechanism turn (e.g. 50.0 for a 50:1 gearbox). With
         * this set, positions and speeds are for the mechanism's output shaft,
         * so Motor1507 works in arm degrees / flywheel RPM, not motor turns.
         */
        public Builder gearRatio(double motorTurnsPerMechanismTurn) {
            this.sensorToMechanismRatio = motorTurnsPerMechanismTurn; return this;
        }

        /**
         * Drum or sprocket pitch diameter for linear mechanisms (elevators), in
         * meters. Lets Motor1507 work in meters and inches.
         */
        public Builder withDrumDiameterMeters(double diameterMeters) {
            this.drumCircumferenceMeters = Math.PI * diameterMeters; return this;
        }

        /** Same as {@link #withDrumDiameterMeters(double)}, in inches. */
        public Builder withDrumDiameterInches(double diameterInches) {
            return withDrumDiameterMeters(diameterInches * INCHES_TO_METERS);
        }

        // ── "At target" tolerances (Motor1507.isAtTarget) ───────────────────

        /** Close enough, in degrees, for position control. */
        public Builder withToleranceDegrees(double degrees) { this.toleranceDegrees = degrees; return this; }

        /** Close enough, in meters, for linear position control. */
        public Builder withToleranceMeters(double meters) { this.toleranceMeters = meters; return this; }

        /** Close enough, in inches, for linear position control. */
        public Builder withToleranceInches(double inches) { return withToleranceMeters(inches * INCHES_TO_METERS); }

        /** Close enough, in RPM, for velocity control. */
        public Builder withToleranceRPM(double rpm) { this.toleranceRpm = rpm; return this; }

        // ── Limits ──────────────────────────────────────────────────────────

        /** Peak output voltage, forward and reverse (reverse is negative). */
        public Builder withVoltageLimits(double fwd, double rev) {
            this.peakForwardVoltage = fwd; this.peakReverseVoltage = rev; return this;
        }

        /** Stator (motor-side) current limit in amps. Limits torque and heat. */
        public Builder withStatorCurrentLimit(double amps) {
            this.statorCurrentLimit = Amps.of(amps); return this;
        }

        /** Stator current limit as a WPILib unit (kept for the swerve paste zone). */
        public Builder withStatorCurrentLimit(Current statorCurrentLimit) {
            this.statorCurrentLimit = statorCurrentLimit; return this;
        }

        /** Supply (battery-side) current limit in amps. The main brownout protection. */
        public Builder withSupplyCurrentLimit(double amps) {
            this.supplyCurrentLimit = Amps.of(amps); return this;
        }

        /** Supply current limit as a WPILib unit (kept for the swerve paste zone). */
        public Builder withSupplyCurrentLimit(Current supplyCurrentLimit) {
            this.supplyCurrentLimit = supplyCurrentLimit; return this;
        }

        /**
         * After the supply limit has been limiting for {@code afterSeconds},
         * drop to {@code amps} until the draw falls below it. Protects the
         * battery (and breaker) from a jammed or stalled mechanism.
         */
        public Builder withSupplyLowerLimit(double amps, double afterSeconds) {
            this.supplyLowerLimitAmps = amps; this.supplyLowerTime = afterSeconds; return this;
        }

        /**
         * Peak torque current (amps) for torque control. Applies together with
         * the stator limit; the lower one wins.
         */
        public Builder withPeakTorqueCurrent(double forwardAmps, double reverseAmps) {
            this.peakForwardTorqueCurrent = forwardAmps;
            this.peakReverseTorqueCurrent = reverseAmps;
            return this;
        }

        /** Forward hardware limit switch: enable, reset position when hit, position to reset to. */
        public Builder withForwardLimit(boolean enable, boolean autoset, double autosetValue) {
            this.forwardLimitEnable = enable;
            this.forwardLimitAutosetEnable = autoset;
            this.forwardLimitAutosetValue = autosetValue;
            return this;
        }

        /** Forward limit switch wiring (normally open or closed). */
        public Builder forwardLimitType(ForwardLimitTypeValue type) {
            this.forwardLimitType = type; return this;
        }

        /** Reverse hardware limit switch: enable, reset position when hit, position to reset to. */
        public Builder withReverseLimit(boolean enable, boolean autoset, double autosetValue) {
            this.reverseLimitEnable = enable;
            this.reverseLimitAutosetEnable = autoset;
            this.reverseLimitAutosetValue = autosetValue;
            return this;
        }

        /** Reverse limit switch wiring (normally open or closed). */
        public Builder reverseLimitType(ReverseLimitTypeValue type) {
            this.reverseLimitType = type; return this;
        }

        // ── Feedback sensor (advanced) ───────────────────────────────────────

        /** Closed-loop sensor: the motor's own encoder (default), a CANcoder, etc. */
        public Builder withFeedbackSensor(FeedbackSensorSourceValue source) {
            this.feedbackSource = source; return this;
        }

        /** CAN ID of the remote feedback sensor. */
        public Builder withRemoteSensorId(int id) {
            this.feedbackRemoteId = id; return this;
        }

        /** Motor turns per sensor turn (used with fused/remote sensors). */
        public Builder withRotorToSensorRatio(double ratio) {
            this.rotorToSensorRatio = ratio; return this;
        }

        /** Sensor turns per mechanism turn. {@link #gearRatio(double)} sets this for the motor's own encoder. */
        public Builder withSensorToMechanismRatio(double ratio) {
            this.sensorToMechanismRatio = ratio; return this;
        }

        /** Absolute sensor offset in rotations (TalonFXS external sensors). */
        public Builder withRotorOffset(double offsetRotations) {
            this.rotorOffset = offsetRotations; return this;
        }

        /** Position wraps around (0 and 1 rotation are the same place). For swerve steering. */
        public Builder withContinuousWrap() {
            this.continuousWrap = true; return this;
        }

        // ── Stall detection ──────────────────────────────────────────────────

        /** Current (amps) above which the motor may be stalled. */
        public Builder withStallCurrentThreshold(double amps) {
            this.stallCurrentThreshold = amps; return this;
        }

        /** Speed (rotations/sec) below which the motor counts as not moving. */
        public Builder withStallVelocityThreshold(double rotationsPerSec) {
            this.stallVelocityThreshold = rotationsPerSec; return this;
        }

        /** How long (seconds) both must hold before it counts as a stall. */
        public Builder withStallTime(double seconds) {
            this.stallTimeSeconds = seconds; return this;
        }

        // ── Simulation ───────────────────────────────────────────────────────

        /** How fast the simulated motor moves toward its target (rotations/sec). */
        public Builder withSimVelocityRps(double rps) {
            this.simVelocityRps = rps; return this;
        }

        // ── CTRE override (advanced) ─────────────────────────────────────────

        /**
         * Direct access to CTRE's TalonFX configuration for anything this class
         * doesn't cover. Runs LAST, after every setting above, so it wins. You
         * can paste CTRE examples in unchanged:
         *
         * <pre>
         *   .withCtreConfig(cfg -> {
         *       cfg.MotorOutput.PeakForwardDutyCycle = 0.8;
         *   })
         * </pre>
         */
        public Builder withCtreConfig(Consumer<TalonFXConfiguration> override) {
            this.ctreFxConfig = override; return this;
        }

        /** Same as {@link #withCtreConfig(Consumer)}, for TalonFXS (Minion) motors. */
        public Builder withCtreFxsConfig(Consumer<TalonFXSConfiguration> override) {
            this.ctreFxsConfig = override; return this;
        }

        // ── Build ────────────────────────────────────────────────────────────

        /** Creates the immutable {@link MotorConfig}. */
        public MotorConfig build() {
            return new MotorConfig(
                preset, slotNumber,
                mode, motorInverted,
                kP, kI, kD,
                kV, kS, kA,
                staticFeedforwardSign,
                kG, gravityType,
                peakForwardVoltage, peakReverseVoltage,
                statorCurrentLimit, supplyCurrentLimit,
                supplyLowerLimitAmps, supplyLowerTime,
                peakForwardTorqueCurrent, peakReverseTorqueCurrent,
                forwardLimitEnable, forwardLimitAutosetEnable, forwardLimitAutosetValue, forwardLimitType,
                reverseLimitEnable, reverseLimitAutosetEnable, reverseLimitAutosetValue, reverseLimitType,
                new Feedback(feedbackSource, feedbackRemoteId, rotorToSensorRatio, sensorToMechanismRatio, rotorOffset),
                drumCircumferenceMeters,
                toleranceDegrees, toleranceMeters, toleranceRpm,
                stallCurrentThreshold, stallVelocityThreshold, stallTimeSeconds,
                brakeMode, continuousWrap, enableFOC,
                simVelocityRps,
                ctreFxConfig, ctreFxsConfig
            );
        }
    }
}
