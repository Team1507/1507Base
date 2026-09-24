//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot;

import org.wpilib.command3.button.CommandGamepad;

import org.team1507.lib.core.framework.LoggedRobot;
import org.team1507.robot.auto.AutoBuilder;
import org.team1507.robot.Constants.RobotMap;
import org.team1507.robot.subsystems.*;

// ─────────────────────────────────────────────────────────────────────────────
// Robot
//
// Holds everything shared by all OpModes: subsystems, controllers, and each
// subsystem's safe default command. What the robot DOES in each mode lives in
// OpMode classes, which WPILib finds automatically and lists in the Driver
// Station:
//   - Autonomous: robot/auto/routines/  (@Autonomous, extend AutoOpMode)
//   - Teleop:     robot/teleop/         (@Teleop) — driver button bindings live here
//
// Commands v3 ties bindings and default commands to the OpMode that creates
// them, and removes them when a different OpMode is selected. So bindings go in
// the OpMode that uses them, and the defaults set here are what every
// subsystem falls back to when no OpMode overrides them.
// ─────────────────────────────────────────────────────────────────────────────
public final class Robot extends LoggedRobot {

    // -------------------------------------------------------------------------
    // Subsystems
    // -------------------------------------------------------------------------

    public final Swerve swerve;

    // TODO(QuestNav 2027): re-add the QuestNavSubsystem field here when QuestNav
    // ships a 2027 build. The parked source is in parked/QuestNavSubsystem.java.txt.

    // -------------------------------------------------------------------------
    // Controllers
    // -------------------------------------------------------------------------

    public final CommandGamepad driver;

    // =========================================================================
    // Constructor
    // =========================================================================

    public Robot() {

        // Subsystems
        swerve = new Swerve();

        // TODO(QuestNav 2027): construct QuestNavSubsystem here, passing
        // swerve::addVisionMeasurement, swerve::resetPose and kQuest.ROBOT_TO_QUEST,
        // then re-add the "Set Pose Left/Start/Right" dashboard commands.

        // Safe defaults: what each subsystem does when no command is using it.
        // OpModes can override these (DriverTeleop sets joystick driving).
        swerve.setDefaultCommand(swerve.idle());

        // Give auto routines access to the subsystems
        AutoBuilder.init(swerve);

        // Controllers (button bindings are made in the OpModes, e.g. DriverTeleop)
        driver = new CommandGamepad(RobotMap.DRIVER_CONTROLLER);
    }
}
