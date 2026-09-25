//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.auto;

import org.team1507.lib.core.auto.RouteDriver;
import org.team1507.robot.Robot;

// ─────────────────────────────────────────────────────────────────────────────
// AutoBuilder
//
// Gives AutoSequence access to the robot, so auto routines never pass
// subsystems around. Robot.java calls AutoBuilder.init(this) once, at the end of
// its constructor.
//
// Every subsystem on the Robot is reachable as AutoBuilder.robot.<name>
// (e.g. AutoBuilder.robot.swerve, AutoBuilder.robot.intake). Adding a subsystem
// to Robot.java is all it takes; there is nothing to register here.
// (Before 2027, each subsystem needed a field and an init() parameter here, and
// forgetting one crashed the auto at enable.)
//
// Students should NEVER instantiate this class; use the static fields.
// ─────────────────────────────────────────────────────────────────────────────
public final class AutoBuilder {

    /** The robot, with every subsystem. Set once by Robot.java. */
    public static Robot robot;

    /**
     * The policy driver, for autos written with {@code new AutoSequence(Driver.POLICY)}.
     * Stays null until the policy code (developed in 1507Labs) installs it; a
     * policy auto without it reports an error and does not drive.
     */
    public static RouteDriver policyDriver = null;

    /** Called ONCE from the Robot.java constructor, after all subsystems are built. */
    public static void init(Robot robot) {
        AutoBuilder.robot = robot;
    }

    // Prevent instantiation — this is a static utility class.
    private AutoBuilder() {}
}
