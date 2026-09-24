//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.logging;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.SchedulerEvent;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;

/**
 * Logs what every command did, so a log can answer "what was the robot doing
 * when the voltage dropped?". LoggedRobot starts it; nothing else needs to.
 *
 * <p>Two things are logged:
 * <ul>
 *   <li>{@code Commands/Events}: one line per change, e.g.
 *       {@code START Intake.run}, {@code END Arm.moveTo},
 *       {@code INTERRUPTED Swerve.idle by Swerve.drive},
 *       {@code CANCELED Auto}, {@code ERROR Arm.moveTo: NullPointerException...}.
 *       In AdvantageScope, put it on a timeline next to battery voltage.</li>
 *   <li>{@code <Subsystem>/Command}: the command using each subsystem right
 *       now ("" when none). When commands are nested (an auto that runs
 *       {@code Swerve.driveTo}), it shows the innermost one; when that ends it
 *       goes back to the outer one.</li>
 * </ul>
 *
 * <p>Uses the Commands v3 scheduler's event listener. v3 reports "mounted"
 * every time a command resumes (every loop), so a command counts as started
 * the first time it's mounted. Commands that were scheduled but replaced before
 * they ever ran are not logged.
 */
public final class CommandLog {

    private static final String EVENTS = "Commands/Events";

    /** Commands that have started and not yet ended. Compared by identity, like the scheduler. */
    private final Set<Command> running = Collections.newSetFromMap(new IdentityHashMap<>());

    /** For each subsystem: its running commands, outermost first. */
    private final Map<Mechanism, List<Command>> stacks = new HashMap<>();
    private final Map<Mechanism, TelemetryTable> tables = new HashMap<>();

    private CommandLog() {}

    /** Starts logging the given scheduler's commands. Called once, by LoggedRobot. */
    public static void start(Scheduler scheduler) {
        // The same event can legitimately repeat (START Intake.run twice in a
        // row after a quick cancel), so keep duplicates for this entry.
        Telemetry.keepDuplicates(EVENTS);
        CommandLog log = new CommandLog();
        scheduler.addEventListener(log::onEvent);
    }

    private void onEvent(SchedulerEvent event) {
        switch (event) {
            case SchedulerEvent.Mounted e -> {
                if (running.add(e.command())) {
                    started(e.command());
                }
            }
            case SchedulerEvent.Completed e ->
                ended(e.command(), "END ");
            case SchedulerEvent.Canceled e ->
                ended(e.command(), "CANCELED ");
            case SchedulerEvent.Interrupted e ->
                ended(e.command(), "INTERRUPTED ", " by " + e.interrupter().name());
            case SchedulerEvent.CompletedWithError e ->
                ended(e.command(), "ERROR ", ": " + e.error());
            default -> { }   // Scheduled (not running yet), Yielded (every loop)
        }
    }

    private void started(Command command) {
        Telemetry.log(EVENTS, "START " + command.name());
        for (Mechanism mechanism : command.requirements()) {
            List<Command> stack = stacks.computeIfAbsent(mechanism, m -> new ArrayList<>());
            stack.add(command);
            logCurrent(mechanism, stack);
        }
    }

    private void ended(Command command, String what) {
        ended(command, what, "");
    }

    private void ended(Command command, String what, String detail) {
        if (!running.remove(command)) {
            return;   // never started: replaced while still queued
        }
        Telemetry.log(EVENTS, what + command.name() + detail);
        for (Mechanism mechanism : command.requirements()) {
            List<Command> stack = stacks.get(mechanism);
            if (stack != null && stack.remove(command)) {
                logCurrent(mechanism, stack);
            }
        }
    }

    private void logCurrent(Mechanism mechanism, List<Command> stack) {
        String current = stack.isEmpty() ? "" : stack.get(stack.size() - 1).name();
        tables.computeIfAbsent(mechanism, m -> Telemetry.getTable(m.getName()))
            .log("Command", current);
    }
}
