//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.framework;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.framework.RobotBase;
import org.wpilib.util.Alert;

import org.team1507.lib.core.impl.ctre.Motor1507;
import org.team1507.lib.core.logging.Telemetry;
import org.team1507.lib.core.util.MotorConfig;

/**
 * Base class for all Team 1507 subsystems. Everything every subsystem needs,
 * so it isn't copied into each one.
 *
 * <h2>Motors: create them with {@link #motor}</h2>
 * <pre>
 *   roller = motor("Roller", Motor1507.Type.FX, RobotMap.INTAKE, kIntake.CONFIG);
 * </pre>
 * That one line creates the motor, names its telemetry under this subsystem
 * ({@code Intake/Roller/...}), and registers it. Every loop, this class then
 * automatically:
 * <ul>
 *   <li><b>refreshes</b> all of the subsystem's motors in one CAN call, BEFORE
 *       {@link #periodic()}, so every motor getter returns fresh values;</li>
 *   <li><b>steps the simulation</b> for each motor (in the simulator only);</li>
 *   <li><b>logs the subsystem's total supply current</b> at
 *       {@code <Subsystem>/TotalSupplyCurrent}, for finding brownouts.</li>
 * </ul>
 * (Before 2027, every subsystem had to collect signal arrays, call refreshAll,
 * and step the sim by hand. Forgetting the refresh meant stale values forever.)
 *
 * <h2>The loop, in order</h2>
 * <ol>
 *   <li>refresh registered motors</li>
 *   <li>{@link #periodic()} — your code: read sensors, log values</li>
 *   <li>log total supply current</li>
 *   <li>simulation only: step registered motors, then {@link #simulationPeriodic()}</li>
 * </ol>
 * All of this runs before any commands, the same as v2 subsystems did.
 *
 * <h2>Also provides</h2>
 * <ul>
 *   <li>{@link #log} — record a value under this subsystem's name</li>
 *   <li>{@link #warnIf} / {@link #faultIf} — an alert on the dashboard while a
 *       problem lasts; {@link #warn} / {@link #fault} — one-time problems</li>
 *   <li>Commands v3: a subsystem is a {@link Mechanism}. Commands that use it
 *       are created with {@code run(...)} / {@code runRepeatedly(...)} and must
 *       be named with {@code .named("Subsystem.action")}.</li>
 * </ul>
 */
public abstract class Subsystem1507 implements Mechanism {

    /** Every Subsystem1507 ever constructed, in construction order. */
    private static final List<Subsystem1507> ALL = new ArrayList<>();

    /** CAN bus for {@link #motor} when none is given. Set once in Robot.java. */
    private static CANBus defaultCanBus = null;

    private final String name;

    /** Motors this subsystem refreshes and simulates (created with {@link #motor}). */
    private final List<Motor1507> managedMotors = new ArrayList<>();
    private BaseStatusSignal[] managedSignals = new BaseStatusSignal[0];

    /** Every motor that counts toward this subsystem's current (managed + tracked). */
    private final List<Motor1507> allMotors = new ArrayList<>();

    /** Dashboard alerts, created once per message and reused. */
    private final Map<String, Alert> alerts = new HashMap<>();

    /** Full telemetry keys ("Name/child"), built once per child and reused. */
    private final Map<String, String> keys = new HashMap<>();

    /**
     * @param name subsystem name — used as the telemetry root for everything
     *             this subsystem and its motors log, and as the name Commands
     *             v3 reports for its commands.
     */
    protected Subsystem1507(String name) {
        this.name = name;
        ALL.add(this);
        Scheduler.getDefault().addPeriodic(this::runEveryLoop);
    }

    /** Returns every subsystem on the robot (used by the failsafe and logging). */
    public static List<Subsystem1507> all() {
        return Collections.unmodifiableList(ALL);
    }

    /**
     * Sets the CAN bus {@link #motor} uses when none is given. Robot.java calls
     * this once, first thing, with {@code Constants.CAN_BUS}. (The library can't
     * read Constants itself: lib/ code must not depend on robot/ code.)
     */
    public static void setDefaultCanBus(CANBus bus) {
        defaultCanBus = bus;
    }

    @Override
    public String getName() {
        return name;
    }

    /** Runs every robot loop, after this subsystem's motors are refreshed. Read sensors and log here. */
    public void periodic() {}

    /** Runs every robot loop in simulation only, after the motors' simulation step. */
    public void simulationPeriodic() {}

    // =========================================================================
    // Motors
    // =========================================================================

    /**
     * Creates a motor on the default CAN bus and registers it with this
     * subsystem, which then refreshes and simulates it every loop.
     *
     * @param motorName name under this subsystem, e.g. "Roller" → {@code Intake/Roller}
     * @param type      FX (Kraken / Falcon) or FXS (Minion)
     * @param canId     CAN ID set in Phoenix Tuner X
     * @param configs   the motor's config (extra configs fill PID slots 1 and 2)
     */
    protected Motor1507 motor(String motorName, Motor1507.Type type, int canId, MotorConfig... configs) {
        if (defaultCanBus == null) {
            throw new IllegalStateException(getName() + ": no default CAN bus. Robot.java must call "
                + "Subsystem1507.setDefaultCanBus(Constants.CAN_BUS) before creating subsystems, "
                + "or pass a CANBus to motor(...).");
        }
        return motor(motorName, type, canId, defaultCanBus, configs);
    }

    /** Same as {@link #motor(String, Motor1507.Type, int, MotorConfig...)}, on a specific CAN bus. */
    protected Motor1507 motor(String motorName, Motor1507.Type type, int canId, CANBus bus, MotorConfig... configs) {
        Motor1507 motor = new Motor1507(key(motorName), type, canId, bus, configs);
        managedMotors.add(motor);
        allMotors.add(motor);

        // Rebuild the signal array once, here, so the loop never allocates.
        List<BaseStatusSignal> signals = new ArrayList<>(List.of(managedSignals));
        signals.addAll(List.of(motor.getSignals()));
        managedSignals = signals.toArray(BaseStatusSignal[]::new);
        return motor;
    }

    /**
     * Counts motors this subsystem did NOT create with {@link #motor} toward its
     * total current, without refreshing or simulating them. For subsystems that
     * already refresh their own motors (Swerve refreshes its motors, CANcoders
     * and gyro together in one call). Most subsystems never need this.
     */
    protected void trackMotors(Motor1507... motors) {
        allMotors.addAll(List.of(motors));
    }

    /** Total current this subsystem's motors are drawing from the battery, in amps. */
    public double getSupplyCurrent() {
        double total = 0.0;
        for (Motor1507 motor : allMotors) {
            total += motor.getSupplyCurrent();
        }
        return total;
    }

    /** The loop described in the class comment. Registered with the scheduler. */
    private void runEveryLoop() {
        if (managedSignals.length > 0) {
            BaseStatusSignal.refreshAll(managedSignals);
        }

        periodic();

        if (!allMotors.isEmpty()) {
            log("TotalSupplyCurrent", getSupplyCurrent());
        }

        if (RobotBase.isSimulation()) {
            for (Motor1507 motor : managedMotors) {
                motor.simulationPeriodic(0.02);
            }
            simulationPeriodic();
        }
    }

    // =========================================================================
    // Telemetry helpers
    // =========================================================================

    /**
     * Returns the full telemetry path for a child field.
     *
     * <p>Example: for a subsystem named {@code "Arm"}, {@code key("AngleDegrees")}
     * returns {@code "Arm/AngleDegrees"}. {@link #motor} uses it so each motor's
     * telemetry nests under the subsystem ({@code Arm/Motor/PositionDeg}, ...).
     *
     * @param child the field or component name to append
     * @return {@code getName() + "/" + child}
     */
    protected String key(String child) {
        // Cached: log() runs every loop, and building "Name/child" each time
        // would create a new string (garbage) on every call.
        String full = keys.get(child);
        if (full == null) {
            full = getName() + "/" + child;
            keys.put(child, full);
        }
        return full;
    }

    /** Records a true/false value at {@code <Subsystem>/<field>}, e.g. {@code log("Stalled", isStalled())}. */
    protected void log(String field, boolean value) {
        Telemetry.set(key(field), value);
    }

    /** Records a number at {@code <Subsystem>/<field>}, e.g. {@code log("AngleDegrees", getAngle())}. */
    protected void log(String field, double value) {
        Telemetry.set(key(field), value);
    }

    /** Records a field position at {@code <Subsystem>/<field>}; AdvantageScope can draw it. */
    protected void log(String field, Pose2d pose) {
        Telemetry.set(key(field), pose);
    }

    /**
     * Records text at {@code <Subsystem>/<field>}. Good for state machine states:
     * <pre>
     *   log("State", state.name());   // e.g. "IDLE", "INTAKING", "STALLED"
     * </pre>
     */
    protected void log(String field, String value) {
        Telemetry.set(key(field), value);
    }

    // =========================================================================
    // Problems: alerts on the dashboard and Driver Station
    // =========================================================================

    /**
     * Shows a MEDIUM (yellow) alert on the dashboard while {@code condition} is
     * true, and clears it when it becomes false. Also reports to the Driver
     * Station each time it turns on. Call it every loop from {@link #periodic()}:
     *
     * <pre>
     *   warnIf(!sensorConnected(), "Beam break sensor unplugged");
     * </pre>
     *
     * Use for problems that come and go and that the driver should know about.
     */
    protected void warnIf(boolean condition, String message) {
        setAlert(condition, message, Alert.Level.MEDIUM);
    }

    /**
     * Same as {@link #warnIf}, as a HIGH (red) alert, for problems that seriously
     * affect the robot (a motor not responding, a mechanism disabled).
     */
    protected void faultIf(boolean condition, String message) {
        setAlert(condition, message, Alert.Level.HIGH);
    }

    /**
     * Reports a one-time warning (e.g. "Pose reset from vision failed"): shown
     * on the Driver Station and left as a MEDIUM alert on the dashboard.
     * For a condition that can recover, use {@link #warnIf} instead so the alert
     * clears itself.
     */
    protected void warn(String message) {
        setAlert(true, message, Alert.Level.MEDIUM);
    }

    /** Same as {@link #warn}, as a HIGH (red) alert. */
    protected void fault(String message) {
        setAlert(true, message, Alert.Level.HIGH);
    }

    /** True if the alert for {@code message} is currently showing. (Used by tests.) */
    boolean isAlertActive(String message) {
        Alert alert = alerts.get(message);
        return alert != null && alert.get();
    }

    private void setAlert(boolean active, String message, Alert.Level level) {
        Alert alert = alerts.get(message);
        if (alert == null) {
            if (!active) {
                return;   // never created an alert for a problem that never happened
            }
            alert = new Alert(getName(), message, "[" + getName() + "] " + message, level);
            alerts.put(message, alert);
        }
        if (active && !alert.get()) {
            // Just turned on: tell the Driver Station once, not every loop.
            String text = "[" + getName() + "] " + message;
            if (level == Alert.Level.HIGH) {
                DriverStationErrors.reportError(text, false);
            } else {
                DriverStationErrors.reportWarning(text, false);
            }
        }
        alert.set(active);
    }
}
