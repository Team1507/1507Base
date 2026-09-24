//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot;

import java.util.ArrayList;
import java.util.List;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;

import org.team1507.lib.core.framework.Subsystem1507;

// ─────────────────────────────────────────────────────────────────────────────
// RobotBehaviors
//
// A single shared library of coordinated, multi-subsystem robot behaviors.
// These are complete robot actions that require two or more subsystems working
// together in sequence or in parallel.
//
// KEY PRINCIPLE:
//   A behavior defined here is the ONE definition used everywhere.
//   Whether triggered by a driver button in teleop or a step in an auto
//   routine, it calls the same method. You never write the same behavior twice.
//
// HOW TELEOP USES THIS:
//   In robot/teleop/DriverTeleop.java:
//     robot.driver.faceUp().onTrue(RobotBehaviors.myBehavior(robot));
//
// HOW AUTO USES THIS:
//   In AutoSequence.java (add a one-line wrapper):
//     public AutoSequence myBehavior() {
//         steps.add(RobotBehaviors.myBehavior(...));
//         return this;
//     }
//   Then in a routine:
//     new AutoSequence().myBehavior().driveToPoint(...).build();
//
// HOW TO ADD A NEW BEHAVIOR (Commands v3):
//   1. Identify which subsystems are involved.
//   2. Write a static method here that combines their commands. The simplest
//      way reads top to bottom:
//
//        return Command.noRequirements(coroutine -> {
//            coroutine.await(elevator.goToHigh());            // wait for it to finish
//            coroutine.awaitAll(arm.score(), intake.eject()); // both at once
//        }).named("ScoreHigh");
//
//      Or chain them:  Command.sequence(a, b).named("...")
//                      Command.parallel(a, b).named("...")
//   3. Add a wrapper in AutoSequence.java if it's needed in auto routines.
//   4. Bind it in an OpMode (e.g. DriverTeleop) if it's a driver control.
//
// NAMING CONVENTION:
//   Name behaviors by what the robot DOES, not what the mechanism is.
//   GOOD:  shoot(), scoreHigh(), intakePiece(), ejectPiece()
//   AVOID: armHighAndIntakeForward(), deployArmRunRoller()
// ─────────────────────────────────────────────────────────────────────────────
public final class RobotBehaviors {

    // Prevent instantiation — this is a static utility class.
    private RobotBehaviors() {}

    // ─────────────────────────────────────────────────────────────────
    // FAILSAFE
    // ─────────────────────────────────────────────────────────────────

    /**
     * Interrupts every command that is using a subsystem.
     *
     * <p>Use when a mechanism appears stuck or unresponsive. This command
     * requires EVERY subsystem at the highest priority, so starting it
     * interrupts whatever was using them. It then finishes immediately, and
     * each subsystem's default command (e.g. joystick driving) starts again on
     * the next loop.
     *
     * <p>Commands that use no subsystem (for example a plain wait) are not affected.
     *
     * <p>Binding (DriverTeleop): {@code robot.driver.back().onTrue(RobotBehaviors.failsafe());}
     */
    public static Command failsafe() {
        List<Mechanism> everything = new ArrayList<>(Subsystem1507.all());
        return Command.requiring(everything)
            .executing(coroutine -> {})
            .withPriority(Command.HIGHEST_PRIORITY)
            .named("Failsafe");
    }
}
