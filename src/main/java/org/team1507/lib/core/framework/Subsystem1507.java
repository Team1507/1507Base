//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.framework;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.command2.SubsystemBase;

import org.team1507.lib.core.logging.Telemetry;

/**
 * Base class for all Team 1507 subsystems.
 *
 * <p>Registers a subsystem name with WPILib (used by dashboards and the
 * command scheduler) and provides two tools for telemetry:
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
public abstract class Subsystem1507 extends SubsystemBase {

    /**
     * @param name subsystem name — used as the NT root for all telemetry
     *             produced by this subsystem and its motors.
     */
    protected Subsystem1507(String name) {
        setName(name); // stored and returned by SubsystemBase.getName()
    }

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
