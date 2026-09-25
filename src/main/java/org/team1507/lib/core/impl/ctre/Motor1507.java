//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.impl.ctre;

import java.util.Collections;
import java.util.EnumMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.hardware.traits.CommonTalon;

import org.wpilib.framework.RobotBase;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryLoggable;
import org.wpilib.telemetry.TelemetryTable;
import org.team1507.lib.core.util.MotorConfig;
import org.team1507.lib.core.util.MotorConfig.ControlMode;

/**
 * One CTRE motor (TalonFX / Kraken, or TalonFXS / Minion) for Team 1507.
 *
 * <p>Students say WHAT they want, in the units they think in, and the motor's
 * {@link MotorConfig} decides HOW (which CTRE control request, gains, limits):
 *
 * <pre>
 *   arm.setPosition(90);            // degrees, at the output shaft
 *   shooter.setRPM(3000);           // or setRPS(50)
 *   elevator.setMeters(0.75);       // or setInches(30), needs a drum diameter
 *   roller.runCurrent(20);          // amps (torque control)
 *   roller.runPercent(0.5);         // -1 to 1
 *   motor.stop();
 *
 *   coroutine.waitUntil(arm::isAtTarget);   // uses the config's tolerance
 *   hopper.resetPosition(0);                // "you are at 0 degrees now"
 * </pre>
 *
 * <p>Reading data: common values have named getters ({@link #getPosition()}
 * in degrees, {@link #getRPM()}, {@link #getSupplyCurrent()}, ...). Every value
 * the motor reports is available through {@link #get(MotorSignal)}. For
 * anything else, {@link #getTalon()} returns the raw CTRE device.
 *
 * <p>Positions and speeds are for the mechanism's OUTPUT SHAFT (the config's
 * {@code gearRatio()} is applied). {@link #getRotorPosition()} and
 * {@link #getRotorRPM()} are the motor itself, before the gearbox.
 *
 * <p>Also provides stall detection ({@link #isStalled()}) and simple simulation.
 */
public final class Motor1507 implements TelemetryLoggable {

    public enum Type { FX, FXS }

    // =========================================================================
    // Signals
    // =========================================================================

    /**
     * Every value the motor reports, in the units students work in. Read any of
     * them with {@link #get(MotorSignal)}. This is also the list of what the
     * motor logs, every loop, under {@code <motor name>/<logName>}.
     *
     * <p>Each has a CAN update rate: position and speed are fast (control and
     * odometry need them); currents and voltages are 50 Hz, so the log gets
     * every sample the motor sends (brownout analysis); the rest are slow to
     * keep the CAN bus from filling up.
     */
    public enum MotorSignal {
        /** Output-shaft position, degrees. */
        POSITION("PositionDeg", "degrees", 100),
        /** Output-shaft speed, RPM. */
        VELOCITY("RPM", "RPM", 100),
        /** Current drawn from the battery, amps. */
        SUPPLY_CURRENT("SupplyCurrent", "A", 50),
        /** Current in the motor windings (proportional to torque), amps. */
        STATOR_CURRENT("StatorCurrent", "A", 50),
        /** Voltage the motor controller is applying to the motor, volts. */
        MOTOR_VOLTAGE("MotorVoltage", "V", 50),
        /** Battery voltage measured at this motor controller, volts. Shows voltage sag during a brownout. */
        SUPPLY_VOLTAGE("SupplyVoltage", "V", 50),
        /** Torque-producing current, amps. */
        TORQUE_CURRENT("TorqueCurrent", "A", 10),
        /** Output as a fraction of full power, -1 to 1. */
        DUTY_CYCLE("DutyCycle", "fraction", 10),
        /** Motor rotor position, before the gearbox, rotations. */
        ROTOR_POSITION("RotorPositionRot", "rotations", 10),
        /** Motor rotor speed, before the gearbox, RPM. */
        ROTOR_VELOCITY("RotorRPM", "RPM", 10),
        /** Motor controller temperature, °C. */
        TEMPERATURE("TemperatureC", "°C", 10),
        /** Closed-loop target, in CTRE units (mechanism rotations, or rotations/s for velocity). */
        CLOSED_LOOP_TARGET("ClosedLoopTarget", "CTRE units", 10),
        /** Closed-loop error, in CTRE units (mechanism rotations, or rotations/s for velocity). */
        CLOSED_LOOP_ERROR("ClosedLoopError", "CTRE units", 10),
        /** Active faults as a bit field (0 = no faults). */
        FAULTS("Faults", "bits", 10),
        /** Faults that happened since they were last cleared, as a bit field. */
        STICKY_FAULTS("StickyFaults", "bits", 10);

        /** Name in the log, e.g. {@code SupplyCurrent} → {@code Intake/Roller/SupplyCurrent}. */
        public final String logName;
        /** Units of the value {@link Motor1507#get(MotorSignal)} returns. */
        public final String units;
        /** How often the motor sends this value over CAN (Hz). */
        public final double updateHz;

        MotorSignal(String logName, String units, double updateHz) {
            this.logName = logName;
            this.units = units;
            this.updateHz = updateHz;
        }

        /** values() copies the array on every call; this copy is made once. */
        private static final MotorSignal[] ALL = values();
    }

    private static final double INCHES_TO_METERS = 0.0254;

    /** Every CAN bus a Motor1507 has been created on (RobotHealthLog logs the CANivores). */
    private static final Set<CANBus> BUSES_IN_USE = new LinkedHashSet<>();

    /** Fallback "close enough" when the config doesn't set a tolerance. */
    private static final double DEFAULT_TOLERANCE_DEGREES = 2.0;
    private static final double DEFAULT_TOLERANCE_RPM = 50.0;

    // =========================================================================
    // Fields
    // =========================================================================

    private final String name;
    /** Where this motor's values are logged ({@code name}/...). */
    private final TelemetryTable table;
    private final ParentDevice device;
    private final CommonTalon talon;
    /** Slot 0 config: control mode, units, tolerances, stall and sim settings. */
    private final MotorConfig config;

    private final Map<MotorSignal, StatusSignal<?>> signals = new EnumMap<>(MotorSignal.class);
    private final BaseStatusSignal[] allSignals;

    // Control requests are created once and reused (CTRE recommendation; no
    // garbage created every loop).
    private final DutyCycleOut dutyRequest;
    private final VoltageOut voltageRequest;
    private final TorqueCurrentFOC torqueRequest;
    private final PositionVoltage positionRequest;
    private final MotionMagicVoltage motionMagicRequest;
    private final VelocityVoltage velocityRequest;
    private final NeutralOut neutralRequest = new NeutralOut();

    // What the motor was last asked to do (for isAtTarget and simulation)
    private enum Target { NONE, POSITION, VELOCITY }
    private Target target = Target.NONE;
    private double targetRotations = 0.0;
    private double targetRps = 0.0;

    // Last targets logged. NaN (no target) never equals itself, so the log's
    // "skip unchanged values" can't catch it; these let log() write NaN once.
    private double lastLoggedTargetPosition = 0.0;
    private double lastLoggedTargetRpm = 0.0;

    // Stall detection
    private double lastNotStalledTime;
    private boolean lastStalled = false;

    // Simulation (mechanism units)
    private double simPositionRotations = 0.0;
    private double simVelocityRps = 0.0;
    private double simStatorCurrent = 0.0;

    // =========================================================================
    // Construction
    // =========================================================================

    /**
     * Creates a motor and applies its configuration.
     *
     * @param name    telemetry name, e.g. "Arm/Motor" (Subsystem1507.motor(...) builds it)
     * @param type    FX (Kraken / Falcon) or FXS (Minion)
     * @param canId   CAN ID set in Phoenix Tuner X
     * @param canBus  CAN bus the motor is wired to (use {@code Constants.CAN_BUS})
     * @param configs motor configuration; extra configs fill PID slots 1 and 2
     */
    public Motor1507(String name, Type type, int canId, CANBus canBus, MotorConfig... configs) {
        BUSES_IN_USE.add(canBus);
        this.name = name;
        this.table = Telemetry.getTable(name);
        this.device = switch (type) {
            case FX  -> new TalonFX(canId, canBus);
            case FXS -> new TalonFXS(canId, canBus);
        };
        this.talon = (CommonTalon) device;
        CtreMotorConfigurator.apply(device, configs);
        this.config = configs[0];

        boolean foc = config.enableFOC();
        dutyRequest        = new DutyCycleOut(0).withEnableFOC(foc);
        voltageRequest     = new VoltageOut(0).withEnableFOC(foc);
        torqueRequest      = new TorqueCurrentFOC(0);
        positionRequest    = new PositionVoltage(0).withEnableFOC(foc);
        motionMagicRequest = new MotionMagicVoltage(0).withEnableFOC(foc);
        velocityRequest    = new VelocityVoltage(0).withEnableFOC(foc);

        signals.put(MotorSignal.POSITION,           talon.getPosition(false));
        signals.put(MotorSignal.VELOCITY,           talon.getVelocity(false));
        signals.put(MotorSignal.SUPPLY_CURRENT,     talon.getSupplyCurrent(false));
        signals.put(MotorSignal.STATOR_CURRENT,     talon.getStatorCurrent(false));
        signals.put(MotorSignal.MOTOR_VOLTAGE,      talon.getMotorVoltage(false));
        signals.put(MotorSignal.TORQUE_CURRENT,     talon.getTorqueCurrent(false));
        signals.put(MotorSignal.DUTY_CYCLE,         talon.getDutyCycle(false));
        signals.put(MotorSignal.ROTOR_POSITION,     talon.getRotorPosition(false));
        signals.put(MotorSignal.ROTOR_VELOCITY,     talon.getRotorVelocity(false));
        signals.put(MotorSignal.SUPPLY_VOLTAGE,     talon.getSupplyVoltage(false));
        signals.put(MotorSignal.TEMPERATURE,        talon.getDeviceTemp(false));
        signals.put(MotorSignal.CLOSED_LOOP_TARGET, talon.getClosedLoopReference(false));
        signals.put(MotorSignal.CLOSED_LOOP_ERROR,  talon.getClosedLoopError(false));
        signals.put(MotorSignal.FAULTS,             talon.getFaultField(false));
        signals.put(MotorSignal.STICKY_FAULTS,      talon.getStickyFaultField(false));

        // (false) above = don't read the value yet. The device may still be
        // booting, and an early read just prints a "frame not received" warning.
        for (MotorSignal s : MotorSignal.ALL) {
            signals.get(s).setUpdateFrequency(s.updateHz);
        }
        allSignals = signals.values().toArray(BaseStatusSignal[]::new);

        // Start the stall timer now, so the first loop can't see a false stall.
        this.lastNotStalledTime = Timer.getTimestamp();
    }

    // =========================================================================
    // Commands to the motor
    // =========================================================================

    /** Runs at a fraction of full power, -1.0 to 1.0. Open loop. */
    public void runPercent(double percent) {
        target = Target.NONE;
        simVelocityRps = percent * config.simVelocityRps();
        simStatorCurrent = 0.0;
        talon.setControl(dutyRequest.withOutput(percent));
    }

    /** Applies a voltage, -12 to 12. Open loop. */
    public void runVoltage(double volts) {
        target = Target.NONE;
        simVelocityRps = volts / 12.0 * config.simVelocityRps();
        simStatorCurrent = 0.0;
        talon.setControl(voltageRequest.withOutput(volts));
    }

    /**
     * Pushes with a fixed torque, given as current in amps (positive =
     * forward). The force stays the same as the load changes, which is why
     * {@code MotorConfig.roller()} uses it. Requires Phoenix Pro.
     */
    public void runCurrent(double amps) {
        target = Target.NONE;
        simVelocityRps = (amps / 20.0) * config.simVelocityRps();
        simStatorCurrent = amps;
        talon.setControl(torqueRequest.withOutput(amps));
    }

    /** Moves the output shaft to an angle in degrees. */
    public void setPosition(double degrees) {
        setPositionRotations(degrees / 360.0);
    }

    /**
     * Moves the output shaft to a position in rotations. Most code should use
     * {@link #setPosition(double)} (degrees); swerve steering uses this.
     */
    public void setPositionRotations(double rotations) {
        target = Target.POSITION;
        targetRotations = rotations;
        simStatorCurrent = 0.0;
        if (config.mode() == ControlMode.MOTION_MAGIC) {
            talon.setControl(motionMagicRequest.withPosition(rotations));
        } else {
            talon.setControl(positionRequest.withPosition(rotations));
        }
    }

    /** Moves a linear mechanism (elevator) to a height in meters. Needs a drum diameter. */
    public void setMeters(double meters) {
        setPositionRotations(meters / drumCircumferenceMeters());
    }

    /** Moves a linear mechanism (elevator) to a height in inches. Needs a drum diameter. */
    public void setInches(double inches) {
        setMeters(inches * INCHES_TO_METERS);
    }

    /** Spins the output shaft at a speed in RPM. */
    public void setRPM(double rpm) {
        setRPS(rpm / 60.0);
    }

    /** Spins the output shaft at a speed in rotations per second. */
    public void setRPS(double rps) {
        target = Target.VELOCITY;
        targetRps = rps;
        simStatorCurrent = 0.0;
        talon.setControl(velocityRequest.withVelocity(rps));
    }

    /** Stops the motor. It brakes or coasts according to its config. */
    public void stop() {
        target = Target.NONE;
        simVelocityRps = 0.0;
        simStatorCurrent = 0.0;
        talon.setControl(neutralRequest);
    }

    /**
     * Tells the motor where it is now, without moving it: "you are at
     * {@code degrees}". Use this to zero a mechanism at a hard stop or limit switch.
     */
    public void resetPosition(double degrees) {
        resetPositionRotations(degrees / 360.0);
    }

    /** Same as {@link #resetPosition(double)}, for linear mechanisms, in meters. */
    public void resetPositionMeters(double meters) {
        resetPositionRotations(meters / drumCircumferenceMeters());
    }

    /** Same as {@link #resetPosition(double)}, for linear mechanisms, in inches. */
    public void resetPositionInches(double inches) {
        resetPositionMeters(inches * INCHES_TO_METERS);
    }

    private void resetPositionRotations(double rotations) {
        simPositionRotations = rotations;
        talon.setPosition(rotations);
    }

    // =========================================================================
    // At target?
    // =========================================================================

    /**
     * True when the last {@code setPosition}/{@code setMeters} or
     * {@code setRPM}/{@code setRPS} target has been reached, within the
     * tolerance from the config ({@code withToleranceDegrees},
     * {@code withToleranceMeters}, {@code withToleranceRPM}). False after an
     * open-loop command ({@code runPercent}, {@code runCurrent}) or {@code stop}.
     */
    public boolean isAtTarget() {
        return switch (target) {
            case POSITION -> Math.abs(getPositionRotations() - targetRotations) <= positionToleranceRotations();
            case VELOCITY -> Math.abs(getRPS() - targetRps) * 60.0 <= velocityToleranceRpm();
            case NONE -> false;
        };
    }

    /** The current position target in degrees, or NaN if there isn't one. */
    public double getTargetPosition() {
        return target == Target.POSITION ? targetRotations * 360.0 : Double.NaN;
    }

    /** The current speed target in RPM, or NaN if there isn't one. */
    public double getTargetRPM() {
        return target == Target.VELOCITY ? targetRps * 60.0 : Double.NaN;
    }

    private double positionToleranceRotations() {
        if (!Double.isNaN(config.toleranceMeters()) && !Double.isNaN(config.drumCircumferenceMeters())) {
            return config.toleranceMeters() / config.drumCircumferenceMeters();
        }
        double degrees = Double.isNaN(config.toleranceDegrees()) ? DEFAULT_TOLERANCE_DEGREES : config.toleranceDegrees();
        return degrees / 360.0;
    }

    private double velocityToleranceRpm() {
        return Double.isNaN(config.toleranceRpm()) ? DEFAULT_TOLERANCE_RPM : config.toleranceRpm();
    }

    // =========================================================================
    // Reading data
    // =========================================================================

    /** Output-shaft position in degrees. */
    public double getPosition() {
        return getPositionRotations() * 360.0;
    }

    /** Output-shaft position in rotations. */
    public double getPositionRotations() {
        if (RobotBase.isSimulation()) return simPositionRotations;
        return signals.get(MotorSignal.POSITION).getValueAsDouble();
    }

    /** Linear position (elevator height) in meters. Needs a drum diameter. */
    public double getMeters() {
        return getPositionRotations() * drumCircumferenceMeters();
    }

    /** Linear position (elevator height) in inches. Needs a drum diameter. */
    public double getInches() {
        return getMeters() / INCHES_TO_METERS;
    }

    /** Output-shaft speed in RPM. */
    public double getRPM() {
        return getRPS() * 60.0;
    }

    /** Output-shaft speed in rotations per second. */
    public double getRPS() {
        if (RobotBase.isSimulation()) return simVelocityRps;
        return signals.get(MotorSignal.VELOCITY).getValueAsDouble();
    }

    /** The motor's own rotor position, before the gearbox, in rotations. */
    public double getRotorPosition() {
        return get(MotorSignal.ROTOR_POSITION);
    }

    /** The motor's own rotor speed, before the gearbox, in RPM. */
    public double getRotorRPM() {
        return get(MotorSignal.ROTOR_VELOCITY);
    }

    /** Current drawn from the battery, in amps. */
    public double getSupplyCurrent() {
        return get(MotorSignal.SUPPLY_CURRENT);
    }

    /** Current in the motor windings (proportional to torque), in amps. */
    public double getStatorCurrent() {
        return get(MotorSignal.STATOR_CURRENT);
    }

    /** Voltage applied to the motor, in volts. */
    public double getMotorVoltage() {
        return get(MotorSignal.MOTOR_VOLTAGE);
    }

    /** Battery voltage measured at this motor controller, in volts. */
    public double getSupplyVoltage() {
        return get(MotorSignal.SUPPLY_VOLTAGE);
    }

    /** Motor controller temperature in °C. */
    public double getTemperature() {
        return get(MotorSignal.TEMPERATURE);
    }

    /**
     * Any value the motor reports, in the units listed on {@link MotorSignal}.
     *
     * <pre>
     *   double temp  = motor.get(MotorSignal.TEMPERATURE);
     *   double error = motor.get(MotorSignal.CLOSED_LOOP_ERROR);
     * </pre>
     */
    public double get(MotorSignal signal) {
        return switch (signal) {
            case POSITION       -> getPosition();
            case VELOCITY       -> getRPM();
            case STATOR_CURRENT -> RobotBase.isSimulation() ? simStatorCurrent
                                   : signals.get(signal).getValueAsDouble();
            case ROTOR_VELOCITY -> signals.get(signal).getValueAsDouble() * 60.0;
            default             -> signals.get(signal).getValueAsDouble();
        };
    }

    /** The motor's config (slot 0). */
    public MotorConfig getConfig() {
        return config;
    }

    /** The telemetry name, e.g. "Arm/Motor". */
    public String getName() {
        return name;
    }

    // =========================================================================
    // Stall detection
    // =========================================================================

    /**
     * True when the motor is pushing hard but not moving: stator current above
     * the config's stall current, speed below its stall velocity, for longer
     * than its stall time. It only DETECTS the stall; your code decides what
     * to do. {@link #log()} records it every loop as {@code Stalled}.
     */
    public boolean isStalled() {
        boolean stalledNow =
            getStatorCurrent() > config.stallCurrentThreshold() &&
            Math.abs(getRPS()) < config.stallVelocityThreshold();

        double now = Timer.getTimestamp();

        if (!stalledNow) {
            lastStalled = false;
            lastNotStalledTime = now;
            return false;
        }

        boolean sustained = (now - lastNotStalledTime) > config.stallTimeSeconds();
        lastStalled = sustained;
        return sustained;
    }

    // =========================================================================
    // CAN signals and the raw device
    // =========================================================================

    /** Refreshes all of this motor's signals in one CAN call. */
    public void refresh() {
        BaseStatusSignal.refreshAll(allSignals);
    }

    /** All of this motor's signals, for batching into a larger refreshAll(). */
    public BaseStatusSignal[] getSignals() {
        return allSignals;
    }

    /**
     * The raw CTRE device, as the interface TalonFX and TalonFXS share. For
     * anything this class doesn't cover. Prefer the methods above.
     */
    public CommonTalon getTalon() {
        return talon;
    }

    /** Every CAN bus any Motor1507 has been created on, in creation order. */
    public static Set<CANBus> busesInUse() {
        return Collections.unmodifiableSet(BUSES_IN_USE);
    }

    /** The raw CTRE device as a ParentDevice (for CAN bus optimization). */
    public ParentDevice getDevice() {
        return device;
    }

    // =========================================================================
    // Simulation
    // =========================================================================

    /**
     * Advances the simple simulation by {@code dt} seconds: speed ramps toward
     * its target, and position moves toward its target at the config's
     * {@code simVelocityRps}. Call every loop in simulation. No-op on a robot.
     */
    public void simulationPeriodic(double dt) {
        if (!RobotBase.isSimulation()) return;
        double rate = config.simVelocityRps();

        switch (target) {
            case POSITION -> {
                if (rate <= 0.0) {
                    simPositionRotations = targetRotations;
                    simVelocityRps = 0.0;
                } else {
                    double error = targetRotations - simPositionRotations;
                    double step = Math.copySign(Math.min(Math.abs(error), rate * dt), error);
                    simPositionRotations += step;
                    simVelocityRps = dt > 1e-6 ? step / dt : 0.0;
                }
            }
            case VELOCITY -> {
                if (rate <= 0.0) {
                    simVelocityRps = targetRps;
                } else {
                    double error = targetRps - simVelocityRps;
                    simVelocityRps += Math.copySign(Math.min(Math.abs(error), rate * dt), error);
                }
                simPositionRotations += simVelocityRps * dt;
            }
            case NONE -> simPositionRotations += simVelocityRps * dt;
        }
    }

    // =========================================================================
    // Telemetry
    // =========================================================================

    /**
     * Logs every {@link MotorSignal}, the current target and the stall state
     * under this motor's name ({@code Intake/Roller/SupplyCurrent}, ...).
     * Subsystem1507 calls this every loop for every motor it knows about, right
     * after the motors are refreshed; students never call it.
     *
     * <p>A value that hasn't changed since the last loop isn't written again,
     * so slow signals (temperature, faults) take almost no space in the log.
     */
    public void log() {
        logTo(table);
    }

    /** Same as {@link #log()}, into any table. Lets {@code Telemetry.log("Name", motor)} work. */
    @Override
    public void logTo(TelemetryTable table) {
        for (MotorSignal s : MotorSignal.ALL) {
            table.log(s.logName, get(s));
        }
        double targetPosition = getTargetPosition();
        if (!(Double.isNaN(targetPosition) && Double.isNaN(lastLoggedTargetPosition))) {
            table.log("TargetPositionDeg", targetPosition);
        }
        lastLoggedTargetPosition = targetPosition;

        double targetRpm = getTargetRPM();
        if (!(Double.isNaN(targetRpm) && Double.isNaN(lastLoggedTargetRpm))) {
            table.log("TargetRPM", targetRpm);
        }
        lastLoggedTargetRpm = targetRpm;

        table.log("Stalled", isStalled());
    }

    @Override
    public String getTelemetryType() {
        return "Motor1507";
    }

    // =========================================================================
    // Internal
    // =========================================================================

    private double drumCircumferenceMeters() {
        double circumference = config.drumCircumferenceMeters();
        if (Double.isNaN(circumference)) {
            throw new IllegalStateException(name + ": meters/inches need a drum diameter in the "
                + "MotorConfig: .withDrumDiameterMeters(...) or .withDrumDiameterInches(...)");
        }
        return circumference;
    }
}
