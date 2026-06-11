//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.BooleanSupplier;

import org.team1507.lib.core.logging.Telemetry;

/**
 * A lightweight command builder for IO‑focused subsystems.
 *
 * <p>CommandBuilder provides a simple, declarative way to construct commands
 * with predictable runtime behavior. All callbacks are stored up front and
 * executed without allocating new objects in the hot path (execute/isFinished).
 *
 * <p>Each command supports:
 * <ul>
 *   <li>Initialization and execution callbacks</li>
 *   <li>Normal finish conditions</li>
 *   <li>Timeout‑based termination</li>
 *   <li>Stall‑based termination</li>
 *   <li>End handlers with full termination‑cause information</li>
 * </ul>
 *
 * <p>Command lifecycle telemetry is emitted automatically, including:
 * <ul>
 *   <li>Start and end timestamps</li>
 *   <li>Active state</li>
 *   <li>Duration</li>
 *   <li>Interruption, timeout, and stall flags</li>
 * </ul>
 *
 * <p>This builder is designed for subsystems that perform direct IO and benefit
 * from deterministic, low‑overhead command execution.
 */
public class CommandBuilder extends Command {

    // -----------------------------
    // Lifecycle callbacks
    // -----------------------------
    private Runnable onInit = () -> {};
    private Runnable onExecute = () -> {};

    /**
     * Handler invoked when the command ends, providing full termination context.
     *
     * <p><b>Important — {@code interrupted} semantics:</b><br>
     * {@code interrupted=true} means the command was <em>externally cancelled</em>:
     * the driver released the button, a failsafe ran, or a competing command took
     * the subsystem. It does NOT mean "something went wrong."
     *
     * <p>When timeout or stall detection fires, the command finishes itself through
     * {@code isFinished()} and WPILib calls {@code end(false)} — so
     * {@code interrupted} will be {@code false} even though the command didn't reach
     * its normal finish condition. Use {@code timedOut} and {@code stalled} to
     * distinguish those cases.
     *
     * <p>Summary:
     * <pre>
     *   Normal finish    → interrupted=false, timedOut=false, stalled=false
     *   Timeout fired    → interrupted=false, timedOut=true,  stalled=false
     *   Stall fired      → interrupted=false, timedOut=false, stalled=true
     *   Button released /
     *   external cancel  → interrupted=true,  timedOut=false, stalled=false
     * </pre>
     */
    @FunctionalInterface
    public interface EndHandler {
        /**
         * @param interrupted true if externally cancelled (button release, failsafe, competing requirement)
         * @param timedOut    true if the command ended due to timeout
         * @param stalled     true if the command ended due to stall detection
         */
        void accept(boolean interrupted, boolean timedOut, boolean stalled);
    }

    private EndHandler onEnd = (i, t, s) -> {};

    // -----------------------------
    // Finish conditions
    // -----------------------------
    private BooleanSupplier finishCondition = () -> false;
    private BooleanSupplier stallCondition = () -> false;

    // -----------------------------
    // Timeout tracking
    // -----------------------------
    private double timeoutSeconds = -1.0;
    private double startTime      =  0.0;
    private boolean timedOut = false;

    // -----------------------------
    // Stall tracking
    // -----------------------------
    private boolean stalled = false;

    // -----------------------------
    // Disabled-mode flag
    // -----------------------------
    private boolean disabledOk = false;

    /**
     * Creates a new CommandBuilder and declares subsystem requirements.
     *
     * @param requirements subsystems required by this command
     */
    public CommandBuilder(Subsystem... requirements) {
        addRequirements(requirements);
    }

    /**
     * Assigns a name to the command for debugging and telemetry.
     *
     * @param name command name
     * @return this builder
     */
    public CommandBuilder named(String name) {
        ensureNotRunning();
        setName(name);
        return this;
    }

    /**
     * Sets the initialization routine, executed once when the command starts.
     *
     * @param r initialization action
     * @return this builder
     */
    public CommandBuilder onInitialize(Runnable r) {
        ensureNotRunning();
        this.onInit = r;
        return this;
    }

    /**
     * Sets the execution routine, executed every scheduler cycle.
     *
     * @param r execution action
     * @return this builder
     */
    public CommandBuilder onExecute(Runnable r) {
        ensureNotRunning();
        this.onExecute = r;
        return this;
    }

    /**
     * Sets the end handler, receiving full termination‑cause information.
     *
     * @param r end handler
     * @return this builder
     */
    public CommandBuilder onEnd(EndHandler r) {
        ensureNotRunning();
        this.onEnd = r;
        return this;
    }

    /**
     * Convenience overload for end handlers that do not use termination causes.
     *
     * @param r action to run when the command ends
     * @return this builder
     */
    public CommandBuilder onEnd(Runnable r) {
        return onEnd((i, t, s) -> r.run());
    }

    /**
     * Sets the normal finish condition for the command.
     *
     * @param condition supplier returning true when the command should end
     * @return this builder
     */
    public CommandBuilder isFinished(BooleanSupplier condition) {
        ensureNotRunning();
        this.finishCondition = condition;
        return this;
    }

    /**
     * Convenience overload for constant finish conditions.
     *
     * @param finished whether the command should finish immediately
     * @return this builder
     */
    public CommandBuilder isFinished(boolean finished) {
        return isFinished(() -> finished);
    }

    /**
     * Sets a timeout for the command. When the timeout expires, the command ends
     * and {@code timedOut} is set to true.
     *
     * @param seconds timeout duration
     * @return this builder
     */
    public CommandBuilder timeout(double seconds) {
        ensureNotRunning();
        this.timeoutSeconds = seconds;
        return this;
    }

    /**
     * Sets a stall finish condition. When the supplier returns true, the command
     * ends and {@code stalled} is set to true.
     *
     * @param stallCondition supplier indicating stall state
     * @return this builder
     */
    public CommandBuilder stallFinish(BooleanSupplier stallCondition) {
        ensureNotRunning();
        this.stallCondition = stallCondition;
        return this;
    }

    /**
     * Allows this command to run while the robot is disabled.
     *
     * <p>Use this for commands that should be triggerable from the dashboard before
     * a match — for example, pose presets that need to run before autonomous starts.
     * Motor output commands should never set this to true.
     *
     * @param value true to allow running when disabled
     * @return this builder
     */
    public CommandBuilder runsWhenDisabled(boolean value) {
        this.disabledOk = value;
        return this;
    }

    /**
     * Equivalent to {@code isFinished(false)} — the command runs until interrupted
     * by the scheduler or ended by a {@code stallFinish} or {@code timeout} condition.
     *
     * <p>Prefer this over {@code .isFinished(false)} at the call site — the intent
     * is immediately clear without having to think about what {@code false} means.
     *
     * @return this builder
     */
    public CommandBuilder runsUntilInterrupted() {
        return isFinished(false);
    }

    /**
     * Registers this command on SmartDashboard so it can be triggered from the
     * driver station dashboard (Elastic, Shuffleboard, etc.).
     *
     * <p>Call this after {@link #named(String)} so the widget label matches the
     * command name. Combine with {@link #runsWhenDisabled(boolean) runsWhenDisabled(true)}
     * for pre-match commands that run while the robot is disabled:
     * <pre>
     *   new CommandBuilder(this)
     *       .named("Set Pose Left")
     *       .onInitialize(...)
     *       .isFinished(true)
     *       .runsWhenDisabled(true)
     *       .publishToDashboard();
     * </pre>
     *
     * @return this builder
     */
    public CommandBuilder publishToDashboard() {
        SmartDashboard.putData(getName(), this);
        return this;
    }

    // -----------------------------
    // WPILib lifecycle
    // -----------------------------

    @Override
    public void initialize() {
        timedOut = false;
        stalled = false;
        startTime = Timer.getFPGATimestamp();

        // Telemetry: command start
        String base = "Command/" + getName();
        Telemetry.set(base + "/Active", true);
        Telemetry.set(base + "/StartTime", startTime);

        onInit.run();
    }

    @Override
    public void execute() {
        onExecute.run();
    }

    @Override
    public void end(boolean interrupted) {
        double endTime = Timer.getFPGATimestamp();
        String base = "Command/" + getName();

        // Telemetry: command end
        Telemetry.set(base + "/Active", false);
        Telemetry.set(base + "/EndTime", endTime);
        Telemetry.set(base + "/Duration", endTime - startTime);

        // Telemetry: termination causes
        Telemetry.set(base + "/Interrupted", interrupted);
        Telemetry.set(base + "/TimedOut", timedOut);
        Telemetry.set(base + "/Stalled", stalled);

        onEnd.accept(interrupted, timedOut, stalled);
    }

    @Override
    public boolean isFinished() {
        // All three conditions (timeout, stall, finish) are checked independently.
        // Multiple flags can become true in the same loop — onEnd should handle any
        // combination with || rather than expecting exactly one true flag.

        // Timeout check
        if (timeoutSeconds >= 0) {
            double elapsed = Timer.getFPGATimestamp() - startTime;
            if (elapsed >= timeoutSeconds) {
                timedOut = true;
                return true;
            }
        }

        // Stall check
        if (stallCondition.getAsBoolean()) {
            stalled = true;
            return true;
        }

        // Normal finish
        return finishCondition.getAsBoolean();
    }

    @Override
    public boolean runsWhenDisabled() {
        return disabledOk;
    }

    // -----------------------------
    // Internal safety
    // -----------------------------
    private void ensureNotRunning() {
        if (isScheduled()) {
            throw new IllegalStateException(
                "Cannot modify a running CommandBuilder"
            );
        }
    }
}
