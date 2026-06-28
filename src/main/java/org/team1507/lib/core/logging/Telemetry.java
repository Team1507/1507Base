//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.logging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.StructArrayPublisher;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Centralized telemetry publisher.
 *
 * <p>{@code Telemetry} manages two forms of logging:
 *
 * <ul>
 *   <li><b>Periodic field logging</b> via {@link InputField}, published at
 *       configurable {@link TelemetryRate}s.</li>
 *   <li><b>Immediate event logging</b> for command lifecycle, stall events,
 *       timestamps, and other one‑shot values.</li>
 * </ul>
 *
 * <p>Periodic logging is rate‑limited and intended for sensor and mechanism
 * state. Event logging writes directly to NetworkTables and is intended for
 * command activity, timestamps, and discrete state changes.
 */
public final class Telemetry {

    private static final Map<String, BooleanPublisher>                       boolPublishers        = new HashMap<>();
    private static final Map<String, DoublePublisher>                        doublePublishers      = new HashMap<>();
    private static final Map<String, StringPublisher>                        stringPublishers      = new HashMap<>();
    private static final Map<String, StructPublisher<Pose2d>>                posePublishers        = new HashMap<>();

    /**
     * Logging period (in seconds) associated with each telemetry rate.
     */
    private static final Map<TelemetryRate, Double> PERIOD_SEC = Map.of(
        TelemetryRate.FAST,   0.02, // 50 Hz
        TelemetryRate.NORMAL, 0.10, // 10 Hz
        TelemetryRate.SLOW,   0.50  // 2 Hz
    );

    /**
     * Registered telemetry fields, grouped by logging rate.
     */
    private static final Map<TelemetryRate, List<InputField<?>>> fields =
        new EnumMap<>(TelemetryRate.class);

    /**
     * Timestamp of the last update for each telemetry rate.
     */
    private static final Map<TelemetryRate, Double> lastUpdateTime =
        new EnumMap<>(TelemetryRate.class);

    private static final Map<String, StructArrayPublisher<SwerveModuleState>> moduleStatePublishers = new HashMap<>();

    private Telemetry() {}

    // -------------------------------------------------------------------------
    // Registration
    // -------------------------------------------------------------------------

    /**
     * Registers an {@link InputField} for periodic telemetry publishing.
     *
     * @param field the telemetry field to register
     */
    public static void register(InputField<?> field) {
        fields
            .computeIfAbsent(field.getRate(), r -> new ArrayList<>())
            .add(field);
    }

    /**
     * Registers multiple {@link InputField}s for periodic telemetry publishing.
     *
     * @param fields the telemetry fields to register
     */
    public static void register(InputField<?>... fields) {
        for (InputField<?> field : fields) {
            register(field);
        }
    }

    // -------------------------------------------------------------------------
    // Event Logging (Immediate)
    // -------------------------------------------------------------------------

    /**
     * Writes a boolean telemetry value immediately.
     *
     * <p>This is intended for command lifecycle flags, stall events, and other
     * discrete state changes that should appear in logs as they occur.
     *
     * @param key   NetworkTables key
     * @param value boolean value to publish
     */
    public static void set(String key, boolean value) {
        BooleanPublisher pub = boolPublishers.get(key);
        if (pub == null) {
            pub = NetworkTableInstance.getDefault().getBooleanTopic(key).publish();
            boolPublishers.put(key, pub);
        }
        pub.set(value);
    }

    /**
     * Writes a string telemetry value immediately.
     *
     * <p>Intended for state machine states and other named status values
     * that should appear in logs as they change.
     *
     * @param key   NetworkTables key
     * @param value string value to publish
     */
    public static void set(String key, String value) {
        StringPublisher pub = stringPublishers.get(key);
        if (pub == null) {
            pub = NetworkTableInstance.getDefault().getStringTopic(key).publish();
            stringPublishers.put(key, pub);
        }
        pub.set(value);
    }

    /**
     * Writes a numeric telemetry value immediately.
     *
     * <p>This is intended for timestamps, durations, and other one‑shot numeric
     * values that should be logged at the moment they occur.
     *
     * @param key   NetworkTables key
     * @param value numeric value to publish
     */
    public static void set(String key, double value) {
        DoublePublisher pub = doublePublishers.get(key);
        if (pub == null) {
            pub = NetworkTableInstance.getDefault().getDoubleTopic(key).publish();
            doublePublishers.put(key, pub);
        }
        pub.set(value);
    }

    /**
     * Publishes a Pose2d as a struct so tools like AdvantageScope
     * can visualize it directly.
     *
     * @param key  NetworkTables key (e.g. "Swerve/Pose")
     * @param pose Pose2d to publish
     */
    public static void set(String key, Pose2d pose) {
        StructPublisher<Pose2d> pub = posePublishers.get(key);

        if (pub == null) {
            // Publish directly at `key` — no extra "/Pose2d" sub-level is added.
            pub = NetworkTableInstance.getDefault()
                .getStructTopic(key, Pose2d.struct).publish();
            posePublishers.put(key, pub);
        }

        pub.set(pose);
    }

    /**
     * Publishes an array of {@link SwerveModuleState} structs so tools like
     * AdvantageScope can visualize module states directly.
     *
     * @param key    NetworkTables key (e.g. "Swerve/ModuleStates")
     * @param states module states to publish
     */
    public static void set(String key, SwerveModuleState[] states) {
        StructArrayPublisher<SwerveModuleState> pub = moduleStatePublishers.get(key);

        if (pub == null) {
            // Use `key` as the full NT path — callers pass "Swerve/ModuleStates",
            // not just "ModuleStates" inside a hardcoded "Swerve" table.
            pub = NetworkTableInstance.getDefault()
                .getStructArrayTopic(key, SwerveModuleState.struct).publish();
            moduleStatePublishers.put(key, pub);
        }

        pub.set(states);
    }

    /**
     * Logs a timestamped event.
     *
     * <p>This convenience method records the current FPGA timestamp under the
     * given key. It is useful for marking command start/end events, stall
     * transitions, and other discrete occurrences.
     *
     * @param key NetworkTables key
     */
    public static void event(String key) {
        set(key, Timer.getFPGATimestamp());
    }

    // -------------------------------------------------------------------------
    // Periodic Publishing
    // -------------------------------------------------------------------------

    /**
     * Publishes registered telemetry fields whose logging period has elapsed.
     *
     * <p>This method should be called periodically (e.g. from
     * {@code robotPeriodic}). Each telemetry rate is evaluated independently.
     */
    public static void update() {
        double now = Timer.getFPGATimestamp();

        for (TelemetryRate rate : TelemetryRate.values()) {
            double last = lastUpdateTime.getOrDefault(rate, 0.0);
            double period = PERIOD_SEC.get(rate);

            if (now - last < period) {
                continue;
            }

            lastUpdateTime.put(rate, now);

            List<InputField<?>> rateFields = fields.get(rate);
            if (rateFields == null) {
                continue;
            }

            for (InputField<?> field : rateFields) {
                field.publish();
            }
        }
    }
}
