//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.util;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import edu.wpi.first.units.measure.Current;

/**
 * Declarative motor configuration for {@link org.team1507.lib.core.impl.ctre.Motor1507}.
 *
 * <p>Describes all hardware settings for a CTRE TalonFX or TalonFXS in a single
 * immutable value. Use {@link #builder()} or {@link #builder(ControlMode)} to
 * construct an instance, then pass it to {@code Motor1507}'s constructor.
 *
 * <p>Multiple {@code MotorConfig} objects can be passed to a single motor to
 * populate several PID slots. The first config (slot 0) also defines all base
 * hardware settings such as current limits, neutral mode, and feedback sensor.
 */
public record MotorConfig(
        int slotNumber,

        ControlMode mode,
        boolean motorInverted,

        double kP, double kI, double kD,
        double kV, double kS, double kA,

        double kG,
        GravityType gravityType,

        double peakForwardVoltage,
        double peakReverseVoltage,

        Current statorCurrentLimit,
        Current supplyCurrentLimit,

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

        // ---------------------------
        // Stall detection thresholds
        // ---------------------------
        double stallCurrentThreshold,
        double stallVelocityThreshold,
        double stallTimeSeconds,

        boolean brakeMode,
        boolean continuousWrap,

        double simVelocityRps
) {

    /**
     * Selects the closed-loop control strategy applied by the configurator and motor.
     *
     * <p>{@code DUTY_CYCLE} and {@code TORQUE} are open-loop — PID and feedforward
     * slot gains are not written to hardware for these modes.
     */
    public static enum ControlMode { DUTY_CYCLE, TORQUE, VELOCITY, POSITION, MOTION_MAGIC }

    public static enum GravityType {
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

    /**
     * Returns a new {@link Builder} with all defaults and {@code DUTY_CYCLE} mode.
     */
    public static Builder builder() {
        return new Builder();
    }

    /**
     * Returns a new {@link Builder} with the given control mode pre-set.
     *
     * @param mode the closed-loop control strategy for this motor
     */
    public static Builder builder(ControlMode mode) {
        return new Builder().mode(mode);
    }

    /**
     * Fluent builder for {@link MotorConfig}.
     *
     * <p>All fields default to safe values (coast mode, no limits enabled, no gains).
     * Call only the methods relevant to your mechanism, then call {@link #build()}.
     */
    public static final class Builder {

        private int slotNumber = 0;

        private ControlMode mode = ControlMode.DUTY_CYCLE;
        private boolean motorInverted = false;

        private double kP = 0, kI = 0, kD = 0;
        private double kV = 0, kS = 0, kA = 0;

        private double kG = 0;
        private GravityType gravityType = GravityType.NONE;

        private double peakForwardVoltage = 12;
        private double peakReverseVoltage = -12;

        private Current statorCurrentLimit = Amps.of(120.0);
        private Current supplyCurrentLimit = Amps.of(70.0);

        // CTRE defaults — 800 A is effectively unlimited.
        // Only set these when using TorqueCurrentFOC (ControlMode.TORQUE).
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

        // ---------------------------
        // Stall defaults
        // ---------------------------
        private double stallCurrentThreshold = 60.0;   // amps
        private double stallVelocityThreshold = 0.2;   // rotations/sec
        private double stallTimeSeconds = 0.25;        // seconds

        private boolean brakeMode = false;

        private boolean continuousWrap = false;

        private double simVelocityRps = 0.0;

        // ============================
        // Builder Methods
        // ============================

        /**
         * Sets the closed-loop control strategy for this motor config.
         *
         * <p>Must match the control request used at runtime (e.g. {@code TORQUE} when
         * calling {@code runTorqueCurrent()}). {@code DUTY_CYCLE} and {@code TORQUE}
         * skip writing PID/feedforward gains to hardware slots.
         *
         * @param m the control mode
         */
        public Builder mode(ControlMode m) { this.mode = m; return this; }

        /**
         * Sets the motor output direction.
         *
         * @param inv {@code true} to invert the motor (clockwise positive),
         *            {@code false} for default (counter-clockwise positive)
         */
        public Builder inverted(boolean inv) { this.motorInverted = inv; return this; }

        /**
         * Sets which PID slot this config populates on the motor controller.
         *
         * <p>Slot 0 is the default and also owns all base hardware settings
         * (current limits, neutral mode, feedback sensor, etc.). Slots 1 and 2
         * carry only gain values.
         *
         * @param slot slot index (0, 1, or 2)
         */
        public Builder slot(int slot) {
            this.slotNumber = slot; return this;
        }

        /**
         * Sets the PID gains for closed-loop control.
         *
         * <p>Ignored for {@code DUTY_CYCLE} and {@code TORQUE} modes.
         *
         * @param p proportional gain
         * @param i integral gain
         * @param d derivative gain
         */
        public Builder withPID(double p, double i, double d) {
            this.kP = p; this.kI = i; this.kD = d; return this;
        }

        /**
         * Sets the feedforward gains for closed-loop control.
         *
         * <p>Ignored for {@code DUTY_CYCLE} and {@code TORQUE} modes.
         *
         * @param kS static friction feedforward (volts)
         * @param kV velocity feedforward (volts per rotation/sec)
         * @param kA acceleration feedforward (volts per rotation/sec²)
         */
        public Builder withFeedforward(double kS, double kV, double kA) {
            this.kS = kS; this.kV = kV; this.kA = kA; return this;
        }

        /**
         * Enables gravity compensation for arm or elevator mechanisms.
         *
         * @param kG   gravity gain (volts) — tune until the mechanism holds position
         *             at rest with no other command
         * @param type compensation shape ({@code COSINE} for arms, {@code CONSTANT}
         *             for elevators)
         */
        public Builder withGravity(double kG, GravityType type) {
            this.kG = kG; this.gravityType = type; return this;
        }

        /**
         * Overrides the peak output voltage in each direction.
         *
         * <p>Defaults to ±12 V (full battery). Reduce to soft-cap roller or
         * conveyor speeds without tuning duty cycle values.
         *
         * @param fwd peak forward voltage (positive, volts)
         * @param rev peak reverse voltage (negative, volts)
         */
        public Builder withVoltageLimits(double fwd, double rev) {
            this.peakForwardVoltage = fwd; this.peakReverseVoltage = rev; return this;
        }

        /**
         * Sets the stator (output) current limit.
         *
         * <p>Stator current is proportional to torque. Limiting it caps the motor's
         * peak torque and protects gearboxes from shock loads. Default is 120 A.
         *
         * @param statorCurrentLimit maximum stator current
         */
        public Builder withStatorCurrentLimit(Current statorCurrentLimit) {
            this.statorCurrentLimit = statorCurrentLimit;
            return this;
        }

        /**
         * Sets the supply (battery) current limit.
         *
         * <p>Supply current is what the battery and breaker see. Limiting it prevents
         * brownouts under sustained load. Default is 70 A.
         *
         * @param supplyCurrentLimit maximum supply current
         */
        public Builder withSupplyCurrentLimit(Current supplyCurrentLimit) {
            this.supplyCurrentLimit = supplyCurrentLimit;
            return this;
        }

        /**
         * Sets peak torque current limits for TorqueCurrentFOC mode (Phoenix Pro).
         *
         * <p>These caps are applied inside the TalonFX alongside the stator current
         * limit — whichever is more restrictive wins. Set these to a value slightly
         * above your commanded torque ({@code TORQUE_AMPS_INTAKE}) as a safety ceiling.
         * Measure actual current draw on the robot first, then set accordingly.
         *
         * @param forwardAmps  max forward stator current (positive, in amps)
         * @param reverseAmps  max reverse stator current (negative, in amps)
         */
        public Builder withPeakTorqueCurrent(double forwardAmps, double reverseAmps) {
            this.peakForwardTorqueCurrent = forwardAmps;
            this.peakReverseTorqueCurrent = reverseAmps;
            return this;
        }

        /**
         * Configures the forward hardware limit switch.
         *
         * @param enable       {@code true} to enable the forward limit switch
         * @param autoset      {@code true} to automatically zero the sensor position
         *                     when the forward limit is triggered
         * @param autosetValue position value to write when auto-zeroing (rotations)
         */
        public Builder withForwardLimit(boolean enable, boolean autoset, double autosetValue) {
            this.forwardLimitEnable = enable;
            this.forwardLimitAutosetEnable = autoset;
            this.forwardLimitAutosetValue = autosetValue;
            return this;
        }

        /**
         * Sets whether the forward limit switch is normally open or normally closed.
         *
         * <p>Default is {@code NormallyOpen}. Only relevant when the forward limit
         * is enabled via {@link #withForwardLimit}.
         *
         * @param type the limit switch wiring type
         */
        public Builder forwardLimitType(ForwardLimitTypeValue type) {
            this.forwardLimitType = type;
            return this;
        }

        /**
         * Configures the reverse hardware limit switch.
         *
         * @param enable       {@code true} to enable the reverse limit switch
         * @param autoset      {@code true} to automatically zero the sensor position
         *                     when the reverse limit is triggered
         * @param autosetValue position value to write when auto-zeroing (rotations)
         */
        public Builder withReverseLimit(boolean enable, boolean autoset, double autosetValue) {
            this.reverseLimitEnable = enable;
            this.reverseLimitAutosetEnable = autoset;
            this.reverseLimitAutosetValue = autosetValue;
            return this;
        }

        /**
         * Sets whether the reverse limit switch is normally open or normally closed.
         *
         * <p>Default is {@code NormallyOpen}. Only relevant when the reverse limit
         * is enabled via {@link #withReverseLimit}.
         *
         * @param type the limit switch wiring type
         */
        public Builder reverseLimitType(ReverseLimitTypeValue type) {
            this.reverseLimitType = type;
            return this;
        }

        /**
         * Sets the feedback sensor source for closed-loop control.
         *
         * <p>Default is {@code RotorSensor} (the internal TalonFX encoder). Use
         * {@code RemoteCANcoder} or similar when fusing an external encoder.
         *
         * @param source the feedback sensor source
         */
        public Builder withFeedbackSensor(FeedbackSensorSourceValue source) {
            this.feedbackSource = source;
            return this;
        }

        /**
         * Sets the CAN ID of a remote feedback sensor (e.g. a CANcoder).
         *
         * <p>Only meaningful when {@link #withFeedbackSensor} is set to a remote
         * source such as {@code RemoteCANcoder} or {@code FusedCANcoder}.
         *
         * @param id CAN device ID of the remote sensor
         */
        public Builder withRemoteSensorId(int id) {
            this.feedbackRemoteId = id;
            return this;
        }

        /**
         * Sets the gear ratio from the motor rotor to the feedback sensor shaft.
         *
         * <p>Used when the sensor is not mounted directly on the rotor (e.g. on the
         * output shaft of a multi-stage gearbox). Default is 1.0 (sensor on rotor).
         *
         * @param ratio rotor rotations per sensor rotation
         */
        public Builder withRotorToSensorRatio(double ratio) {
            this.rotorToSensorRatio = ratio;
            return this;
        }

        /**
         * Sets the gear ratio from the feedback sensor shaft to the mechanism output.
         *
         * <p>Setting this allows position and velocity setpoints to be expressed in
         * mechanism units (e.g. arm degrees, elevator inches) rather than raw sensor
         * rotations. Default is 1.0.
         *
         * @param ratio sensor rotations per mechanism output rotation
         */
        public Builder withSensorToMechanismRatio(double ratio) {
            this.sensorToMechanismRatio = ratio;
            return this;
        }

        /**
         * Applies a fixed offset to the rotor position at startup (rotations).
         *
         * <p>Useful for mechanisms with a known home position that differs from
         * the rotor's power-on reading. Default is 0.0.
         *
         * @param offsetRotations rotor position offset in rotations
         */
        public Builder withRotorOffset(double offsetRotations) {
            this.rotorOffset = offsetRotations;
            return this;
        }

        // ---------------------------
        // Stall builder methods
        // ---------------------------

        /**
         * Sets the stator current above which the motor is considered loaded for
         * stall detection purposes.
         *
         * <p>Default is 60 A. Tune this above the motor's free-spinning current
         * but below the current it draws when mechanically jammed.
         *
         * @param amps stall current threshold in amps
         */
        public Builder withStallCurrentThreshold(double amps) {
            this.stallCurrentThreshold = amps;
            return this;
        }

        /**
         * Sets the rotor velocity below which the motor is considered not moving for
         * stall detection purposes.
         *
         * <p>Default is 0.2 rotations/sec. Increase if the mechanism moves slowly
         * under normal load and triggers false stall detections.
         *
         * @param rotationsPerSec stall velocity threshold in rotations per second
         */
        public Builder withStallVelocityThreshold(double rotationsPerSec) {
            this.stallVelocityThreshold = rotationsPerSec;
            return this;
        }

        /**
         * Sets how long the stall condition must persist before {@code isStalled()}
         * returns {@code true}.
         *
         * <p>Default is 0.25 seconds. Increase to debounce transient current spikes;
         * decrease for faster stall detection on sensitive mechanisms.
         *
         * @param seconds minimum stall duration in seconds
         */
        public Builder withStallTime(double seconds) {
            this.stallTimeSeconds = seconds;
            return this;
        }

        /** Puts the motor in brake mode — rotor actively resists motion when stopped. */
        public Builder withBrake() {
            this.brakeMode = true;
            return this;
        }

        /** Puts the motor in coast mode — rotor spins freely when stopped. */
        public Builder withCoast() {
            this.brakeMode = false;
            return this;
        }

        /**
         * Enables continuous wrap for position closed-loop control.
         *
         * <p>When enabled, the controller treats the position as a continuous circle
         * (e.g. 0–1 rotation) and always takes the shortest path to the target.
         * Use for mechanisms like swerve steer that can rotate freely.
         */
        public Builder withContinuousWrap() {
            this.continuousWrap = true;
            return this;
        }

        /**
         * Configures the simulation slew rate for position-controlled motors.
         *
         * <p>The motor's simulated position will move toward its target at this speed
         * (rotations per second) instead of jumping instantly. For velocity and duty
         * cycle modes, this value scales the simulated velocity proportionally.
         * Default is 0.0 (instant).
         *
         * @param rps simulated slew rate in rotations per second
         *            (e.g. 0.0833 ≈ 30°/sec with a 1:1 sensor-to-mechanism ratio)
         */
        public Builder withSimVelocityRps(double rps) {
            this.simVelocityRps = rps;
            return this;
        }

        // ============================
        // Build
        // ============================

        /**
         * Constructs the immutable {@link MotorConfig} from the current builder state.
         *
         * @return a fully configured {@code MotorConfig} ready to pass to
         *         {@link org.team1507.lib.core.impl.ctre.Motor1507}
         */
        public MotorConfig build() {
            return new MotorConfig(
                slotNumber,
                mode, motorInverted,
                kP, kI, kD,
                kV, kS, kA,
                kG, gravityType,
                peakForwardVoltage, peakReverseVoltage,

                statorCurrentLimit,
                supplyCurrentLimit,

                peakForwardTorqueCurrent,
                peakReverseTorqueCurrent,

                forwardLimitEnable,
                forwardLimitAutosetEnable,
                forwardLimitAutosetValue,
                forwardLimitType,

                reverseLimitEnable,
                reverseLimitAutosetEnable,
                reverseLimitAutosetValue,
                reverseLimitType,

                new Feedback(
                    feedbackSource,
                    feedbackRemoteId,
                    rotorToSensorRatio,
                    sensorToMechanismRatio,
                    rotorOffset
                ),

                stallCurrentThreshold,
                stallVelocityThreshold,
                stallTimeSeconds,

                brakeMode,

                continuousWrap,

                simVelocityRps
            );
        }
    }
}
