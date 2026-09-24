//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.framework;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;

import org.team1507.robot.RobotBehaviors;

// ─────────────────────────────────────────────────────────────────────────────
// CommandsV3PatternsTest
//
// Commands v3 is new (alpha) and its API may change. These tests run the
// command patterns 1507Base relies on through the real v3 scheduler, so an
// update that changes their behavior fails the build instead of the robot:
//
//   - Subsystem1507.periodic() runs every loop (v3 has no built-in periodic)
//   - A wrapper that keeps a step's requirements can run that step
//     (AutoSequence.withName() and .withDebug() depend on this)
//   - Sequences of wrapped steps on the same subsystem run in order
//   - A deadline group cancels its optional commands (AutoSequence.deadline())
//   - RobotBehaviors.failsafe() interrupts running commands, and the default
//     command comes back afterward
// ─────────────────────────────────────────────────────────────────────────────
class CommandsV3PatternsTest {

    /** A do-nothing subsystem that counts its periodic() calls. */
    private static final class TestSubsystem extends Subsystem1507 {
        int periodicCalls = 0;

        TestSubsystem() {
            super("TestSubsystem");
        }

        @Override
        public void periodic() {
            periodicCalls++;
        }
    }

    private final Scheduler scheduler = Scheduler.getDefault();

    @BeforeAll
    static void initHal() {
        HAL.initialize();
    }

    @BeforeEach
    void resetScheduler() {
        scheduler.cancelAll();
    }

    private void runLoops(int loops) {
        for (int i = 0; i < loops; i++) {
            scheduler.run();
        }
    }

    /** A command on the subsystem that runs for a number of loops, recording its name when it finishes. */
    private static Command steps(TestSubsystem subsystem, String name, int loops, List<String> log) {
        return subsystem.run(coroutine -> {
            for (int i = 0; i < loops; i++) {
                coroutine.yield();
            }
            log.add(name);
        }).named(name);
    }

    @Test
    void subsystemPeriodicRunsEveryLoop() {
        TestSubsystem subsystem = new TestSubsystem();
        runLoops(5);
        assertEquals(5, subsystem.periodicCalls);
    }

    @Test
    void wrapperKeepingRequirementsRunsTheStep() {
        TestSubsystem subsystem = new TestSubsystem();
        List<String> log = new ArrayList<>();
        Command step = steps(subsystem, "step", 3, log);

        // Same pattern as AutoSequence.withName() / timed()
        Command wrapper = Command.requiring(step.requirements())
            .executing(coroutine -> coroutine.await(step))
            .named("wrapper");

        scheduler.schedule(wrapper);
        runLoops(10);

        assertEquals(List.of("step"), log);
        assertFalse(scheduler.isScheduledOrRunning(wrapper));
    }

    @Test
    void sequenceOfWrappedStepsRunsInOrder() {
        TestSubsystem subsystem = new TestSubsystem();
        List<String> log = new ArrayList<>();
        Command first = steps(subsystem, "first", 2, log);
        Command second = steps(subsystem, "second", 2, log);

        Command sequence = Command.sequence(
            Command.noRequirements(coroutine -> log.add("timer")).named("startTimer"),
            Command.requiring(first.requirements()).executing(c -> c.await(first)).named("A"),
            Command.requiring(second.requirements()).executing(c -> c.await(second)).named("B")
        ).withAutomaticName();

        scheduler.schedule(sequence);
        runLoops(20);

        assertEquals(List.of("timer", "first", "second"), log);
    }

    @Test
    void deadlineCancelsOptionalCommands() {
        TestSubsystem subsystem = new TestSubsystem();
        List<String> log = new ArrayList<>();
        boolean[] optionalCanceled = { false };

        Command deadline = Command.waitUntil(() -> log.size() >= 0).named("deadline");
        Command forever = subsystem.runRepeatedly(() -> {})
            .whenCanceled(() -> optionalCanceled[0] = true)
            .named("forever");

        // Same shape as AutoSequence.deadline()
        Command group = Command.parallel(deadline).optional(forever).named("group");

        scheduler.schedule(group);
        runLoops(5);

        assertFalse(scheduler.isScheduledOrRunning(group));
        assertTrue(optionalCanceled[0]);
    }

    @Test
    void failsafeInterruptsAndDefaultComesBack() {
        TestSubsystem subsystem = new TestSubsystem();
        subsystem.setDefaultCommand(subsystem.idle());

        boolean[] canceled = { false };
        Command forever = subsystem.runRepeatedly(() -> {})
            .whenCanceled(() -> canceled[0] = true)
            .named("forever");

        scheduler.schedule(forever);
        runLoops(2);
        assertTrue(scheduler.isRunning(forever));

        scheduler.schedule(RobotBehaviors.failsafe());
        runLoops(3);

        assertTrue(canceled[0], "failsafe should interrupt the running command");
        assertFalse(scheduler.isRunning(forever));
        assertTrue(
            scheduler.getRunningCommandsFor(subsystem).stream()
                .anyMatch(c -> c.name().equals("TestSubsystem[IDLE]")),
            "default command should run again after the failsafe");
    }
}
