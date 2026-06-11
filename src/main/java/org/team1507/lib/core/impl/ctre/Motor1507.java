//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.impl.ctre;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

import org.team1507.lib.core.logging.InputField;
import org.team1507.lib.core.logging.Telemetry;
import org.team1507.lib.core.logging.TelemetryRate;
import org.team1507.lib.core.util.MotorConfig;

/**
 * Unified CTRE motor abstraction for Team 1507.
 *
 * <p>{@code Motor1507} owns motor hardware, control, telemetry, and
 * mechanism‑agnostic stall detection. Configuration is applied declaratively
 * via {@link MotorConfig}.
 */
public final class Motor1507 {

    public enum Type { FX, FXS }

    private final Object motor;
    private final CtreMotorSignals signals;
    private final String name;

    // ------------------------------------------------------------
    // Stall detection fields
    // ------------------------------------------------------------

    /** Current threshold above which the motor is considered loaded. */
    private final double stallCurrentThreshold;

    /** Velocity threshold below which the motor is considered not moving. */
    private final double stallVelocityThreshold;

    /** Minimum duration the stall condition must persist. */
    private final double stallTimeSeconds;

    /** Timestamp of the last moment the motor was not stalled. */
    private double lastNotStalledTime;

    /** True when the motor was considered stalled on the previous loop iteration. */
    private boolean lastStalled = false;

    // ------------------------------------------------------------
    // Telemetry fields
    // ------------------------------------------------------------

    public final InputField<Double> rotorPosition;
    public final InputField<Double> rotorVelocity;
    public final InputField<Double> supplyCurrent;
    public final InputField<Double> statorCurrent;
    public final InputField<Double> motorVoltage;
    public final InputField<Double> deviceTemp;

    // ------------------------------------------------------------
    // Simulated Data
    // ------------------------------------------------------------

    private double simRotorPosition   = 0.0;
    private double simRotorVelocity   = 0.0;
    private double simTargetRotations = Double.NaN;
    private double simTargetVelocity  = Double.NaN;
    private double simStatorCurrent   = 0.0;
    private final double simVelocityRps;
    private double simLastTimestamp = -1.0;

    /**
     * Creates a new motor instance and applies the provided configuration.
     *
     * @param name    human-readable motor name (e.g. "Feeder", "IntakeArm")
     * @param type    motor hardware type
     * @param canId   CAN device ID
     * @param configs motor configuration specifications
     */
    public Motor1507(String name, Type type, int canId, MotorConfig... configs) {
        this.name = name;
        this.motor = createMotor(type, canId);
        CtreMotorConfigurator.apply(motor, configs);

        this.signals = CtreMotorSignals.fromMotor(motor);

        // Extract stall thresholds and sim config from slot 0
        MotorConfig base = configs[0];
        this.stallCurrentThreshold = base.stallCurrentThreshold();
        this.stallVelocityThreshold = base.stallVelocityThreshold();
        this.stallTimeSeconds = base.stallTimeSeconds();
        this.simVelocityRps = base.simVelocityRps();

        // Initialize to now so the stall timer starts from a valid baseline.
        // Leaving this at 0 would make (now - 0) = robot uptime on the first loop,
        // immediately triggering a false stall on every motor at boot.
        this.lastNotStalledTime = Timer.getFPGATimestamp();

        // --------------------------------------------------------
        // Declare telemetry
        // --------------------------------------------------------

        rotorPosition = new InputField<>(
            key("Input", "Position"),
            this::getRotorPosition,
            TelemetryRate.NORMAL
        );

        rotorVelocity = new InputField<>(
            key("Input", "Velocity"),
            this::getRotorVelocity,
            TelemetryRate.FAST
        );

        supplyCurrent = new InputField<>(
            key("Input", "SupplyCurrent"),
            signals::getSupplyCurrent,
            TelemetryRate.NORMAL
        );

        statorCurrent = new InputField<>(
            key("Input", "StatorCurrent"),
            signals::getStatorCurrent,
            TelemetryRate.NORMAL
        );

        motorVoltage = new InputField<>(
            key("Input", "Voltage"),
            signals::getMotorVoltage,
            TelemetryRate.SLOW
        );

        deviceTemp = new InputField<>(
            key("Input", "Temperature"),
            signals::getDeviceTemp,
            TelemetryRate.SLOW
        );

        Telemetry.register(
            rotorPosition,
            rotorVelocity,
            supplyCurrent,
            statorCurrent,
            motorVoltage,
            deviceTemp
        );
    }

    // ============================================================
    // CONTROL
    // ============================================================

    private void setControl(ControlRequest request) {
        if (motor instanceof TalonFX fx) {
            fx.setControl(request);
        } else if (motor instanceof TalonFXS fxs) {
            fxs.setControl(request);
        }
    }

    /** Runs the motor at an open-loop duty cycle ({@code -1.0} to {@code +1.0}). */
    public void runDuty(double dutyCycle) {
        simTargetVelocity = Double.NaN;
        simStatorCurrent  = 0.0;
        simRotorVelocity  = dutyCycle * simVelocityRps;
        setControl(new DutyCycleOut(dutyCycle));
    }

    /**
     * Commands the motor to a target stator current (torque) in amps.
     *
     * <p>Requires Phoenix Pro. Unlike {@link #runDuty}, this mode delivers a
     * fixed torque regardless of mechanical load — current draw does not
     * increase as back-pressure rises, preventing brownouts on loaded rollers.
     *
     * @param amps target stator current; positive = forward, negative = reverse
     */
    public void runTorqueCurrent(double amps) {
        simTargetVelocity = Double.NaN;
        simStatorCurrent  = amps;
        simRotorVelocity  = (amps / 20.0) * simVelocityRps;
        setControl(new TorqueCurrentFOC(amps));
    }

    /** Commands the motor to a target position using duty-cycle closed-loop. */
    public void setPositionDuty(double rotations) {
        setControl(new PositionDutyCycle(rotations));
    }

    /** Commands the motor to a target position using voltage closed-loop. */
    public void setPositionVoltage(double rotations) {
        simTargetRotations = rotations;
        simStatorCurrent   = 0.0;
        setControl(new PositionVoltage(rotations));
    }

    /** Commands the motor to a target position using voltage closed-loop with an additional feedforward. */
    public void setPositionVoltage(double rotations, double ffVolts) {
        simTargetRotations = rotations;
        simStatorCurrent   = 0.0;
        setControl(
            new PositionVoltage(rotations)
                .withFeedForward(ffVolts)
        );
    }

    /** Commands the motor to a target velocity in rotations per second using voltage closed-loop. */
    public void setVelocityRPS(double motorRPS) {
        simTargetVelocity = motorRPS;
        simStatorCurrent  = 0.0;
        setControl(new VelocityVoltage(motorRPS));
    }

    /** Commands the motor to a target velocity in rotations per second with an additional feedforward. */
    public void setVelocityRPS(double motorRPS, double ffVolts) {
        simTargetVelocity = motorRPS;
        simStatorCurrent  = 0.0;
        setControl(
            new VelocityVoltage(motorRPS)
                .withFeedForward(ffVolts)
        );
    }

    /** Stops the motor and clears any active control request. */
    public void stop() {
        simRotorVelocity   = 0.0;
        simTargetVelocity  = Double.NaN;
        simTargetRotations = Double.NaN;
        simStatorCurrent   = 0.0;
        if (motor instanceof TalonFX fx) {
            fx.stopMotor();
        } else if (motor instanceof TalonFXS fxs) {
            fxs.stopMotor();
        }
    }

    /**
     * Advances the simulated velocity toward its target at the configured
     * {@code simVelocityRps} acceleration rate. Call from the owning subsystem's
     * {@code simulationPeriodic()} every loop. No-op on real hardware.
     *
     * @param dt loop period in seconds (typically 0.02)
     */
    public void simulationPeriodic(double dt) {
        if (!RobotBase.isSimulation() || Double.isNaN(simTargetVelocity)) return;
        if (simVelocityRps <= 0.0) {
            simRotorVelocity = simTargetVelocity;
            return;
        }
        double maxDelta = simVelocityRps * dt;
        double error    = simTargetVelocity - simRotorVelocity;
        simRotorVelocity += Math.copySign(Math.min(Math.abs(error), maxDelta), error);
    }

    // ============================================================
    // OBSERVATION
    // ============================================================

    public double getRotorPosition() {
        if (RobotBase.isSimulation()) {
            double now = Timer.getFPGATimestamp();
            double dt = (simLastTimestamp < 0) ? 0.02 : now - simLastTimestamp;
            simLastTimestamp = now;

            if (!Double.isNaN(simTargetRotations) && simVelocityRps > 0) {
                double maxStep = simVelocityRps * dt;
                double error = simTargetRotations - simRotorPosition;
                double actualStep = Math.copySign(Math.min(Math.abs(error), maxStep), error);
                simRotorPosition += actualStep;
                simRotorVelocity = (dt > 1e-6) ? actualStep / dt : 0.0;
            } else {
                simRotorPosition += simRotorVelocity * dt;
            }
            return simRotorPosition;
        }
        return signals.getRotorPosition();
    }

    public double getRotorVelocity() {
        if (RobotBase.isSimulation()) {
            return simRotorVelocity;
        }
        return signals.getRotorVelocity();
    }

    /** Returns the motor's supply current in amps. */
    public double getSupplyCurrent() {
        return signals.getSupplyCurrent();
    }

    /** Returns the motor's stator current in amps. */
    public double getStatorCurrent() {
        if (RobotBase.isSimulation()) return simStatorCurrent;
        return signals.getStatorCurrent();
    }

    /** Returns the motor output voltage in volts. */
    public double getMotorVoltage() {
        return signals.getMotorVoltage();
    }

    /** Returns the motor controller's device temperature in degrees Celsius. */
    public double getDeviceTemp() {
        return signals.getDeviceTemp();
    }

    /** Refreshes all CAN status signals for this motor in one bus call. */
    public void refresh() {
        signals.refresh();
    }

    /**
     * Returns the underlying Phoenix 6 hardware device.
     *
     * <p>Used for CAN bus optimization (e.g., {@code ParentDevice.optimizeBusUtilizationForAll}).
     * Avoid using the raw device for control — use the methods on this class instead.
     */
    public com.ctre.phoenix6.hardware.ParentDevice getDevice() {
        return (com.ctre.phoenix6.hardware.ParentDevice) motor;
    }

    /** Returns all tracked status signals for this motor, for use in batched refresh calls. */
    public com.ctre.phoenix6.BaseStatusSignal[] getSignals() {
        return signals.getSignals();
    }

    /**
     * Returns whether the motor is physically stalled.
     *
     * <p>A stall is defined as:
     * <ul>
     *   <li>stator current above the configured threshold</li>
     *   <li>rotor velocity below the configured threshold</li>
     *   <li>condition persisting longer than the configured stall time</li>
     * </ul>
     *
     * <p>This method is mechanism‑agnostic and does not interpret the meaning
     * of a stall. Subsystems should provide semantic interpretation.
     *
     * <p><b>Signal freshness:</b> {@code getStatorCurrent()} and {@code getRotorVelocity()}
     * read from cached CAN signals. Call this only after {@code BaseStatusSignal.refreshAll()}
     * (or the subsystem's {@code periodic()}) has run, otherwise stall detection
     * may lag by one 20 ms loop.
     *
     * @return true if the motor is stalled
     */
    public boolean isStalled() {
        boolean stalledNow =
            getStatorCurrent() > stallCurrentThreshold &&
            Math.abs(getRotorVelocity()) < stallVelocityThreshold;

        double now = Timer.getFPGATimestamp();

        if (!stalledNow) {
            if (lastStalled) {
                Telemetry.event(name + "/StallEnd");
            }
            lastStalled = false;
            lastNotStalledTime = now;
            return false;
        }

        boolean sustained = (now - lastNotStalledTime) > stallTimeSeconds;

        if (sustained && !lastStalled) {
            Telemetry.event(name + "/StallStart");
        }

        lastStalled = sustained;
        return sustained;
    }

    // ============================================================
    // INTERNAL
    // ============================================================

    private String key(String category, String field) {
        return name + "/" + category + "/" + field;
    }

    private static Object createMotor(Type type, int canId) {
        return switch (type) {
            case FX  -> new TalonFX(canId);
            case FXS -> new TalonFXS(canId);
        };
    }
}
