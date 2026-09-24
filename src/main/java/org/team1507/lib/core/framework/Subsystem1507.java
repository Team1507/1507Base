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
import java.util.List;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.framework.RobotBase;

import org.team1507.lib.core.logging.Telemetry;

/**
 * Base class for all Team 1507 subsystems.
 *
 * <p>2027: a subsystem is a Commands v3 {@link Mechanism}. Commands that use it
 * are created with {@code run(...)} / {@code runRepeatedly(...)} and must be
 * named with {@code .named("Subsystem.action")}.
 *
 * <p>Mechanisms have no built-in {@code periodic()} in Commands v3, so this
 * class registers {@link #periodic()} (and {@link #simulationPeriodic()} in
 * simulation) with the scheduler. They run every loop, before any commands,
 * the same as v2 subsystems did.
 *
 * <p>It also provides two tools for telemetry:
 *
 * <ul>
 *   <li>{@link #key(String)} — builds the fully-qualified NT path for a child
 *       field, e.g. {@code key("AngleDegrees")} returns {@code "Arm/AngleDegrees"}.
 *       Pass this to {@link Motor1507} constructors so motor inputs nest under
 *       the subsystem in AdvantageScope.</li>
 *   <li>{@link #log(String, boolean)}, {@link #log(String, double)},
 *       {@link #log(String, Pose2d)} — convenience wrappers that call
 *       {@link Telemetry#set} with the auto-prefixed key, removing the
 *       need to repeat the subsystem name in every call site.</li>
 * </ul>
 *
 * <p>Usage in a subsystem:
 * <pre>
 *   // Motor names include the full NT path so inputs nest under the subsystem:
 *   motor = new Motor1507(key("Motor"), Motor1507.Type.FX, id, Constants.CAN_BUS, config);
 *
 *   // periodic() calls use log() instead of Telemetry.set():
 *   log("AngleDegrees", getCurrentAngle());
 *   log("Stalled", isStalled());
 * </pre>
 */
public abstract class Subsystem1507 implements Mechanism {

    /** Every Subsystem1507 ever constructed, in construction order. */
    private static final List<Subsystem1507> ALL = new ArrayList<>();

    private final String name;

    /**
     * @param name subsystem name — used as the NT root for all telemetry
     *             produced by this subsystem and its motors, and as the
     *             mechanism name shown in command logs.
     */
    protected Subsystem1507(String name) {
        this.name = name;
        ALL.add(this);
        Scheduler.getDefault().addPeriodic(() -> {
            periodic();
            if (RobotBase.isSimulation()) {
                simulationPeriodic();
            }
        });
    }

    /** Returns every subsystem on the robot (used by the failsafe and logging). */
    public static List<Subsystem1507> all() {
        return Collections.unmodifiableList(ALL);
    }

    @Override
    public String getName() {
        return name;
    }

    /** Runs every robot loop, before commands. Override to read sensors and log. */
    public void periodic() {}

    /** Runs every robot loop in simulation only, after {@link #periodic()}. */
    public void simulationPeriodic() {}

    // =========================================================================
    // Telemetry helpers
    // =========================================================================

    /**
     * Returns the fully-qualified NT path for a child field.
     *
     * <p>Example: for a subsystem named {@code "Arm"},
     * {@code key("AngleDegrees")} returns {@code "Arm/AngleDegrees"}.
     *
     * <p>Use this when constructing motors so their input fields nest under
     * this subsystem in AdvantageScope:
     * <pre>
     *   new Motor1507(key("MotorA"), Motor1507.Type.FX, id, Constants.CAN_BUS, config)
     *   // → Motor1507 publishes at Arm/MotorA/Input/Position, etc.
     * </pre>
     *
     * @param child the field or component name to append
     * @return fully-qualified NT path: {@code getName() + "/" + child}
     */
    protected String key(String child) {
        return getName() + "/" + child;
    }

    /**
     * Publishes a boolean value at the auto-prefixed NT key.
     * Equivalent to {@code Telemetry.set(key(field), value)}.
     *
     * @param field the field name (e.g. {@code "Stalled"})
     * @param value the value to publish
     */
    protected void log(String field, boolean value) {
        Telemetry.set(key(field), value);
    }

    /**
     * Publishes a double value at the auto-prefixed NT key.
     * Equivalent to {@code Telemetry.set(key(field), value)}.
     *
     * @param field the field name (e.g. {@code "AngleDegrees"})
     * @param value the value to publish
     */
    protected void log(String field, double value) {
        Telemetry.set(key(field), value);
    }

    /**
     * Publishes a Pose2d struct at the auto-prefixed NT key.
     * Equivalent to {@code Telemetry.set(key(field), pose)}.
     *
     * @param field the field name (e.g. {@code "Pose"})
     * @param pose  the pose to publish
     */
    protected void log(String field, Pose2d pose) {
        Telemetry.set(key(field), pose);
    }

    /**
     * Publishes a string value at the auto-prefixed NT key.
     * Equivalent to {@code Telemetry.set(key(field), value)}.
     *
     * <p>Use this to log state machine states or any other named status:
     * <pre>
     *   log("State", state.name());   // e.g. "IDLE", "INTAKING", "STALLED"
     * </pre>
     *
     * @param field the field name (e.g. {@code "State"})
     * @param value the string value to publish
     */
    protected void log(String field, String value) {
        Telemetry.set(key(field), value);
    }

    // =========================================================================
    // Driver Station reporting
    // =========================================================================

    /**
     * Reports a warning to the Driver Station with the subsystem name prepended.
     *
     * <p>Use for recoverable conditions the driver should know about:
     * hardware momentarily unavailable, a sensor reading outside expected range,
     * a state machine falling back to a safe mode, etc.
     *
     * <p>Example:
     * <pre>
     *   warn("Tracking lost — check headset view.");
     *   // DS shows: "[QuestNav] Tracking lost — check headset view."
     * </pre>
     *
     * @param message description of the warning condition
     */
    protected void warn(String message) {
        DriverStationErrors.reportWarning("[" + getName() + "] " + message, false);
    }

    /**
     * Reports an error to the Driver Station with the subsystem name prepended.
     *
     * <p>Use for serious conditions that will degrade robot function:
     * a motor that failed to configure, a sensor returning invalid data,
     * a state machine unable to proceed, etc.
     *
     * <p>Example:
     * <pre>
     *   fault("Motor A is not responding — arm disabled.");
     *   // DS shows: "[Arm] Motor A is not responding — arm disabled."
     * </pre>
     *
     * @param message description of the fault condition
     */
    protected void fault(String message) {
        DriverStationErrors.reportError("[" + getName() + "] " + message, false);
    }
}
