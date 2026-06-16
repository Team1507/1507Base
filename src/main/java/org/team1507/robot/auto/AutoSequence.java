//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.auto;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.team1507.lib.core.logging.Telemetry;
import org.team1507.lib.core.util.Alliance;
import org.team1507.robot.auto.nodes.FieldFlip;

// ─────────────────────────────────────────────────────────────────────────────
// AutoSequence
//
// Fluent builder for constructing autonomous routines. Each method appends a
// Command to an internal list, and build() assembles them into a single
// sequential Command that runs step by step.
//
// BASIC USAGE:
//   new AutoSequence()
//       .startTimer()
//       .resetPose(Nodes.Robot.Start.RIGHT)
//       .driveToPoint(Nodes.Robot.Score.RIGHT, true)
//       .stop()
//       .build();
//
// SPEED MODIFIERS:
//   Speed modifiers must appear immediately before a motion command.
//   They apply to that one step only and then reset automatically.
//
//   .slow().driveToPoint(Nodes.Robot.Pickup.APPROACH_RIGHT, true)
//   .creep().driveToPoint(Nodes.Robot.Pickup.STATION_RIGHT, true)
//
// DEBUG TELEMETRY:
//   Add .withDebug() anywhere in the chain to enable per-step NT timing.
//   Name individual steps with .withName("label") chained after any step method.
//
//   new AutoSequence()
//       .withDebug()
//       .startTimer()
//       .moveThroughBy(bump, 0.2, 2.0).withName("Cross bump")
//       .build();
//
// GROUPS (parallel / race / deadline):
//   Each branch inside a group is its own mini-sequence, written as a lambda:
//       seq -> seq.step1().step2()
//
//   Each branch shares the root autoTimer so timer-gated steps work correctly
//   inside groups.
//
//   Examples:
//     .parallel(
//         seq -> seq.mySubsystemCommand(),
//         seq -> seq.driveForwardMeters(1.0, true)
//     )
//     .race(
//         seq -> seq.driveToPoint(Nodes.Robot.Score.RIGHT, true),
//         seq -> seq.waitSeconds(2.0)
//     )
//     .deadline(
//         seq -> seq.driveToPoint(Nodes.Robot.Pickup.APPROACH_RIGHT, true),  // deadline
//         seq -> seq.mySubsystemCommand()                                     // runs alongside
//     )
//
// HOW TO ADD NEW AUTO STEPS (each year):
//   1. Add your command to AutoBuilder.java (or RobotBehaviors.java if multi-subsystem).
//   2. Add a one-line wrapper method here following the pattern of existing methods.
//   3. Use it in your routine file.
// ─────────────────────────────────────────────────────────────────────────────
public final class AutoSequence {

    // -------------------------------------------------------------------------
    // Core Fields
    // -------------------------------------------------------------------------

    private final List<Command> steps = new ArrayList<>();

    // Speed state — consumed by the next motion command, then reset.
    // Always set with .withSpeed(), .slow(), or .creep() immediately before
    // the motion step you want it to apply to.
    private Double nextSpeedOverride   = null;
    private Double nextAngularOverride = null;

    // When true, build() wraps every root step with per-step timing telemetry.
    // Set via .withDebug(). Off by default — no NT overhead in production.
    private boolean debugEnabled = false;

    // Autonomous match timer — started with .startTimer(), read by .waitUntilTime() etc.
    // Passed into branch sub-sequences so timer-gated steps work correctly inside groups.
    private final Timer autoTimer;


    // -------------------------------------------------------------------------
    // Constructors
    // -------------------------------------------------------------------------

    /**
     * Creates a new AutoSequence builder.
     * No arguments needed — all subsystems are accessed through AutoBuilder.
     */
    public AutoSequence() {
        this.autoTimer = new Timer();
    }

    /** Package-private: shares the root sequence's timer with a branch sub-sequence. */
    AutoSequence(Timer sharedTimer) {
        this.autoTimer = sharedTimer;
    }


    // =========================================================================
    // DEBUG / NAMING
    // =========================================================================

    /**
     * Enables per-step timing telemetry for this sequence.
     * Without this call, build() produces no NetworkTables output (zero overhead).
     *
     * Place anywhere in the chain — it takes effect when build() is called:
     *   new AutoSequence().withDebug().startTimer().resetPose(start)...build();
     */
    public AutoSequence withDebug() {
        this.debugEnabled = true;
        return this;
    }

    /**
     * Overrides the telemetry name of the most recently added step.
     * Chain immediately after any step method:
     *
     *   .moveThroughBy(bump, 0.2, 2.0).withName("Cross bump")
     *
     * Has no effect if debug is not enabled.
     */
    public AutoSequence withName(String name) {
        if (!steps.isEmpty()) {
            steps.get(steps.size() - 1).setName(name);
        }
        return this;
    }


    // =========================================================================
    // SPEED MODIFIERS
    //
    // These do NOT add a step. They set a temporary override that the NEXT
    // motion command will consume. After that command runs, the override resets.
    //
    // Rule: always place a speed modifier immediately before a motion command.
    //   CORRECT:   .slow().driveToPoint(target, true)
    //   INCORRECT: .slow().myStep().driveToPoint(...)  ← slow is wasted on myStep
    // =========================================================================

    /**
     * Applies a custom translational speed to the next motion step (m/s).
     * Angular rate uses the default.
     */
    public AutoSequence withSpeed(double speedMetersPerSec) {
        this.nextSpeedOverride   = speedMetersPerSec;
        this.nextAngularOverride = AutoBuilder.swerve.getMaxAngular();
        return this;
    }

    /**
     * Applies a custom translational AND angular speed to the next motion step.
     */
    public AutoSequence withSpeed(double speedMetersPerSec, double angularRadPerSec) {
        this.nextSpeedOverride   = speedMetersPerSec;
        this.nextAngularOverride = angularRadPerSec;
        return this;
    }

    /**
     * Moderately slow movement — 50% speed, 75% angular rate.
     * Good for approach paths where precision matters.
     */
    public AutoSequence slow() {
        this.nextSpeedOverride   = AutoBuilder.swerve.getMaxSpeed() * 0.5;
        this.nextAngularOverride = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        return this;
    }

    /**
     * Very slow, precise movement — 30% speed, 50% angular rate.
     * Good for final alignment steps or tight corridor navigation.
     */
    public AutoSequence creep() {
        this.nextSpeedOverride   = AutoBuilder.swerve.getMaxSpeed() * 0.3;
        this.nextAngularOverride = RotationsPerSecond.of(0.50).in(RadiansPerSecond);
        return this;
    }

    // Internal helpers — read and clear the speed/angular overrides for one step.
    private double consumeSpeed() {
        double speed = (nextSpeedOverride != null) ? nextSpeedOverride : AutoBuilder.swerve.getMaxSpeed();
        nextSpeedOverride   = null;
        nextAngularOverride = null;
        return speed;
    }

    private double consumeAngular() {
        double angular = (nextAngularOverride != null) ? nextAngularOverride : AutoBuilder.swerve.getMaxAngular();
        nextAngularOverride = null;
        return angular;
    }


    // =========================================================================
    // DRIVE COMMANDS
    // =========================================================================

    /**
     * Resets the robot's field pose to the given Pose2d.
     * Always call this as the first step in an auto routine.
     * Automatically flips the pose for Red alliance.
     */
    public AutoSequence resetPose(Pose2d pose) {
        steps.add(AutoBuilder.swerve.resetPoseCommand(Alliance.isRed() ? FieldFlip.pose(pose) : pose));
        return this;
    }

    /** Drives forward a fixed distance along the robot's current heading. */
    public AutoSequence driveForwardMeters(double distanceMeters, boolean stopAtEnd) {
        steps.add(AutoBuilder.swerve.driveForwardMeters(distanceMeters, consumeSpeed(), stopAtEnd));
        return this;
    }

    /** Drives to a field pose and optionally stops on arrival. Flips for Red alliance. */
    public AutoSequence driveToPoint(Pose2d target, boolean stopAtEnd) {
        steps.add(AutoBuilder.swerve.driveToPoint(Alliance.isRed() ? FieldFlip.pose(target) : target, consumeSpeed(), stopAtEnd));
        return this;
    }

    /** Drives to a field pose and stops on arrival. Convenience alias for driveToPoint(target, true). */
    public AutoSequence driveTo(Pose2d target) {
        steps.add(AutoBuilder.swerve.driveToPoint(Alliance.isRed() ? FieldFlip.pose(target) : target, consumeSpeed(), true));
        return this;
    }

    /**
     * Passes through a waypoint at constant speed without stopping.
     * Finishes when the robot enters passRadius meters of the waypoint.
     */
    public AutoSequence moveThrough(Pose2d waypoint, double passRadius) {
        double angular = consumeAngular();
        double speed   = consumeSpeed();
        steps.add(AutoBuilder.swerve.moveThroughPose(Alliance.isRed() ? FieldFlip.pose(waypoint) : waypoint, speed, angular, passRadius));
        return this;
    }

    /**
     * Drives to a field pose, but cancels if autoTimer passes cutoffSeconds.
     * Whichever comes first ends the step. Good for time-budgeted paths.
     */
    public AutoSequence driveToBy(Pose2d goal, double cutoffSeconds) {
        double speed = consumeSpeed();
        steps.add(Commands.race(
            AutoBuilder.swerve.driveToPoint(Alliance.isRed() ? FieldFlip.pose(goal) : goal, speed, true),
            Commands.waitUntil(() -> autoTimer.get() >= cutoffSeconds)
        ).withName("driveToBy"));
        return this;
    }

    /**
     * Passes through a waypoint, but cancels if autoTimer passes cutoffSeconds.
     * Whichever comes first ends the step. Good for time-budgeted waypoint chains.
     */
    public AutoSequence moveThroughBy(Pose2d waypoint, double passRadius, double cutoffSeconds) {
        double angular = consumeAngular();
        double speed   = consumeSpeed();
        steps.add(Commands.race(
            AutoBuilder.swerve.moveThroughPose(Alliance.isRed() ? FieldFlip.pose(waypoint) : waypoint, speed, angular, passRadius),
            Commands.waitUntil(() -> autoTimer.get() >= cutoffSeconds)
        ).withName("moveThroughBy"));
        return this;
    }

    /** Follows a PathPlanner path file to completion. Path files live in deploy/pathplanner/paths/. */
    public AutoSequence drivePath(String pathName) {
        try {
            steps.add(com.pathplanner.lib.auto.AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName))
                .withName("drivePath_" + pathName));
        } catch (Exception e) {
            throw new RuntimeException("Failed to load PathPlanner path: " + pathName, e);
        }
        return this;
    }

    /** Follows a PathPlanner path, but cancels if autoTimer passes cutoffSeconds. */
    public AutoSequence drivePathBy(String pathName, double cutoffSeconds) {
        try {
            steps.add(Commands.race(
                com.pathplanner.lib.auto.AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName)),
                Commands.waitUntil(() -> autoTimer.get() >= cutoffSeconds)
            ).withName("drivePathBy_" + pathName));
        } catch (Exception e) {
            throw new RuntimeException("Failed to load PathPlanner path: " + pathName, e);
        }
        return this;
    }

    /** Rotates in place to face the given field pose. Robot position does not change. */
    public AutoSequence pointToTarget(Pose2d target) {
        steps.add(AutoBuilder.swerve.pointToTarget(Alliance.isRed() ? FieldFlip.pose(target) : target));
        return this;
    }

    /** Rotates to face a target heading in degrees. Adds 180° automatically for Red alliance. */
    public AutoSequence changeHeading(double headingDeg) {
        steps.add(AutoBuilder.swerve.changeHeading(Alliance.isRed() ? headingDeg + 180.0 : headingDeg));
        return this;
    }

    /** Snaps to the heading stored in the given pose's rotation component. */
    public AutoSequence changeHeading(Pose2d pose) {
        steps.add(AutoBuilder.swerve.changeHeading(Alliance.isRed() ? FieldFlip.pose(pose) : pose));
        return this;
    }

    /** Stops all swerve modules. */
    public AutoSequence stop() {
        steps.add(AutoBuilder.swerve.stopCommand());
        return this;
    }


    // =========================================================================
    // ROBOT BEHAVIORS
    //
    // Multi-subsystem coordinated actions defined in RobotBehaviors.java.
    // These are the same commands used for teleop button bindings.
    // Add a one-line wrapper here for each behavior you want available in auto.
    //
    // Example:
    //   public AutoSequence myBehavior() {
    //       steps.add(RobotBehaviors.myBehavior());
    //       return this;
    //   }
    // =========================================================================


    // =========================================================================
    // HOW TO ADD NEW SUBSYSTEM STEPS (each year)
    //
    // For a positional subsystem (reaches a setpoint, then finishes):
    //   public AutoSequence elevatorHigh() {
    //       steps.add(AutoBuilder.elevator.goToCommand(Setpoint.HIGH));
    //       return this;
    //   }
    //
    // For a free-running subsystem (runs until interrupted):
    //   public AutoSequence feederFeed() {
    //       steps.add(AutoBuilder.feeder.feedCommand());
    //       return this;
    //   }
    //
    // For a multi-subsystem behavior from RobotBehaviors:
    //   public AutoSequence ejectPiece() {
    //       steps.add(RobotBehaviors.ejectPiece());
    //       return this;
    //   }
    // =========================================================================


    // =========================================================================
    // GROUPS: PARALLEL / RACE / DEADLINE
    //
    // Each group takes one or more Branch lambdas. A Branch is a lambda that
    // receives a fresh AutoSequence and adds steps to it:
    //
    //     seq -> seq.step1().step2()
    //
    // "seq" is a new AutoSequence for that branch — it is NOT the outer sequence.
    // You can chain as many steps as you want inside a branch.
    // Speed modifiers work normally inside branches.
    // Each branch shares the root autoTimer so timer-gated steps work correctly.
    //
    // PARALLEL  — all branches run at the same time, ends when ALL are done.
    // RACE      — all branches run at the same time, ends when the FIRST finishes.
    // DEADLINE  — all branches run at the same time, ends when the FIRST ARGUMENT
    //             (the deadline branch) finishes. All others are cancelled.
    // =========================================================================

    /**
     * Runs all branches simultaneously. Ends when ALL branches finish.
     *
     * Example:
     *   .parallel(
     *       seq -> seq.mySubsystemCommand(),
     *       seq -> seq.driveForwardMeters(1.0, true)
     *   )
     */
    public AutoSequence parallel(Branch... branches) {
        List<Command> commands = new ArrayList<>();
        for (Branch branch : branches) {
            AutoSequence sub = new AutoSequence(this.autoTimer);
            branch.build(sub);
            commands.add(sub.buildRaw());
        }
        steps.add(Commands.parallel(commands.toArray(Command[]::new)).withName("parallel"));
        return this;
    }

    /**
     * Runs all branches simultaneously. Ends when the FIRST branch finishes,
     * cancelling all others.
     *
     * Example:
     *   .race(
     *       seq -> seq.driveToPoint(Nodes.Robot.Score.RIGHT, true),
     *       seq -> seq.waitSeconds(2.0)
     *   )
     */
    public AutoSequence race(Branch... branches) {
        List<Command> commands = new ArrayList<>();
        for (Branch branch : branches) {
            AutoSequence sub = new AutoSequence(this.autoTimer);
            branch.build(sub);
            commands.add(sub.buildRaw());
        }
        steps.add(Commands.race(commands.toArray(Command[]::new)).withName("race"));
        return this;
    }

    /**
     * Runs all branches simultaneously. The FIRST branch is the deadline —
     * when it finishes, all other branches are cancelled.
     *
     * Use this when you want one action to set the duration and everything
     * else runs alongside it.
     *
     * Example:
     *   .deadline(
     *       seq -> seq.driveToPoint(Nodes.Robot.Pickup.APPROACH_RIGHT, true),  // deadline
     *       seq -> seq.mySubsystemCommand()                                     // runs alongside
     *   )
     */
    public AutoSequence deadline(Branch deadlineBranch, Branch... others) {
        AutoSequence deadlineSeq = new AutoSequence(this.autoTimer);
        deadlineBranch.build(deadlineSeq);

        List<Command> otherCommands = new ArrayList<>();
        for (Branch branch : others) {
            AutoSequence sub = new AutoSequence(this.autoTimer);
            branch.build(sub);
            otherCommands.add(sub.buildRaw());
        }

        steps.add(Commands.deadline(
            deadlineSeq.buildRaw(),
            otherCommands.toArray(Command[]::new)
        ).withName("deadline"));
        return this;
    }


    // =========================================================================
    // TIMER UTILITIES
    //
    // The auto timer lets you gate actions on match time rather than duration.
    // Always call .startTimer() as your first step if you plan to use
    // .waitUntilTime(), .driveToBy(), or .moveThroughBy().
    // =========================================================================

    /**
     * Starts the autonomous match timer.
     * Call this as the very first step in any routine that uses timer-gated steps.
     */
    public AutoSequence startTimer() {
        steps.add(Commands.runOnce(() -> {
            autoTimer.reset();
            autoTimer.start();
        }).withName("startTimer"));
        return this;
    }

    /**
     * Waits until the auto timer reaches a specific match time (in seconds).
     * Useful for precisely aligning actions to the match clock.
     */
    public AutoSequence waitUntilTime(double matchTimeSeconds) {
        steps.add(Commands.waitUntil(() -> autoTimer.get() >= matchTimeSeconds).withName("waitUntilTime"));
        return this;
    }

    /**
     * Returns a Command that completes when the timer reaches a given time.
     * Use this to gate custom commands not yet wrapped in AutoSequence.
     *
     * Example:
     *   .addCommand(Commands.race(
     *       AutoBuilder.mySubsystem.runCommand(),
     *       endAtTime(13.5)
     *   ))
     */
    public Command endAtTime(double matchTimeSeconds) {
        return Commands.waitUntil(() -> autoTimer.get() >= matchTimeSeconds);
    }


    // =========================================================================
    // UTILITY
    // =========================================================================

    /** Waits a fixed number of seconds before proceeding to the next step. */
    public AutoSequence waitSeconds(double seconds) {
        steps.add(Commands.waitSeconds(seconds).withName("waitSeconds"));
        return this;
    }

    /**
     * Waits until a condition becomes true before proceeding.
     * Use with method references: .waitUntil(mySubsystem::isReady)
     */
    public AutoSequence waitUntil(BooleanSupplier condition) {
        steps.add(Commands.waitUntil(condition).withName("waitUntil"));
        return this;
    }

    /**
     * Adds any Command directly into the sequence.
     * Use this to insert commands that don't yet have a named wrapper method here.
     *
     * Example:
     *   .addCommand(RobotBehaviors.myBehavior())
     */
    public AutoSequence addCommand(Command command) {
        steps.add(command);
        return this;
    }


    // =========================================================================
    // BUILD
    // =========================================================================

    /**
     * Builds the branch command sequence with NO per-step timing wrapper.
     * Used internally by parallel/race/deadline so only the root build() logs timing.
     */
    Command buildRaw() {
        return Commands.sequence(steps.toArray(Command[]::new));
    }

    /**
     * Builds the final autonomous Command.
     * Call this at the END of every routine's build() method.
     * Per-step telemetry is only active when .withDebug() was called.
     */
    public Command build() {
        if (!debugEnabled) {
            return buildRaw();
        }
        Command[] timedSteps = new Command[steps.size()];
        for (int i = 0; i < steps.size(); i++) {
            final Command step = steps.get(i);
            final String key   = String.format("Auto/Step%02d_%s", i, sanitize(step.getName()));
            final double[] t   = { 0.0 };
            timedSteps[i] = step
                .beforeStarting(() -> {
                    t[0] = autoTimer.get();
                    Telemetry.set(key + "/Active",    true);
                    Telemetry.set(key + "/StartTime", t[0]);
                })
                .finallyDo(interrupted -> {
                    double dur = autoTimer.get() - t[0];
                    Telemetry.set(key + "/Active",      false);
                    Telemetry.set(key + "/Duration",    dur);
                    Telemetry.set(key + "/Interrupted", interrupted);
                });
        }
        return Commands.sequence(timedSteps);
    }

    private static String sanitize(String name) {
        return name.replaceAll("[^A-Za-z0-9_]", "_");
    }


    // =========================================================================
    // BRANCH INTERFACE
    //
    // A Branch is a lambda that configures a sub-AutoSequence for use inside
    // .parallel(), .race(), or .deadline().
    //
    // Usage:  seq -> seq.step1().step2()
    //
    // "seq" is automatically created — you just chain steps on it.
    // The result is compiled into a sequential Command for that branch.
    // =========================================================================

    /**
     * Functional interface for defining a branch inside a group.
     *
     * Written as a lambda:  seq -> seq.step1().step2()
     */
    @FunctionalInterface
    public interface Branch {
        void build(AutoSequence seq);
    }
}