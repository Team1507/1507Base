//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.logging;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.Objects;
import java.util.function.Supplier;

/**
 * A typed telemetry signal descriptor.
 *
 * <p>An {@code InputField} represents a named value that can be published by the
 * telemetry system. It associates a fully-qualified telemetry key with a value
 * supplier and a desired logging rate.
 *
 * <p>Values are retrieved on demand via the provided {@link Supplier} when the
 * telemetry system publishes data.
 *
 * @param <T> the type of value produced by this field
 */
public final class InputField<T> {

    /** Fully-qualified telemetry key (e.g. "Shooter/TopMotor/Velocity"). */
    private final String key;

    /** Supplier that provides the current value at publish time. */
    private final Supplier<T> supplier;

    /** Desired logging rate hint for this field. */
    private final TelemetryRate rate;

    /** Cached NetworkTables entry for this field. */
    private final NetworkTableEntry entry;

    /**
     * Creates a new {@code InputField} with a default logging rate of
     * {@link TelemetryRate#SLOW}.
     *
     * @param key      fully-qualified telemetry key
     * @param supplier supplier that provides the current value
     */
    public InputField(String key, Supplier<T> supplier) {
        this(key, supplier, TelemetryRate.SLOW);
    }

    /**
     * Creates a new {@code InputField} with an explicit logging rate.
     *
     * @param key      fully-qualified telemetry key
     * @param supplier supplier that provides the current value
     * @param rate     desired logging rate hint
     */
    public InputField(String key, Supplier<T> supplier, TelemetryRate rate) {
        this.key = Objects.requireNonNull(key, "key");
        this.supplier = Objects.requireNonNull(supplier, "supplier");
        this.rate = Objects.requireNonNull(rate, "rate");

        this.entry = NetworkTableInstance
            .getDefault()
            .getEntry(this.key);
    }

    /**
     * Publishes the current value of this field to NetworkTables.
     */
    public void publish() {
        entry.setValue(supplier.get());
    }

    /**
     * Returns the telemetry key associated with this field.
     *
     * @return the fully-qualified telemetry key
     */
    public String getKey() {
        return key;
    }

    /**
     * Returns the desired logging rate for this field.
     *
     * @return the telemetry rate hint
     */
    public TelemetryRate getRate() {
        return rate;
    }
}
