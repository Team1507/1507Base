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
import org.team1507.lib.core.framework.Subsystem1507;
import org.team1507.robot.auto.AutoBuilder;
import org.team1507.robot.auto.PoseSeeds;
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

        // Every motor created with Subsystem1507.motor(...) goes on this CAN bus
        // unless it names another one. Must come before creating subsystems.
        Subsystem1507.setDefaultCanBus(Constants.CAN_BUS);

        // Log the PDH's per-channel currents (for tracing brownouts).
        // TODO(SEASON LOGGING-2, LOGGING-3): on the real robot, check the logs reach a
        // USB drive, and check loop time and CAN load (Season Setup Checklist).
        logPowerDistribution(Constants.PDH_CAN_PORT);

        // Subsystems
        swerve = new Swerve();

        // TODO(QuestNav 2027): construct QuestNavSubsystem here, passing
        // swerve::addVisionMeasurement, swerve::resetPose and kQuest.ROBOT_TO_QUEST,
        // then re-add the "Set Pose Left/Start/Right" dashboard commands.

        // Safe defaults: what each subsystem does when no command is using it.
        // OpModes can override these (DriverTeleop sets joystick driving).
        swerve.setDefaultCommand(swerve.idle());

        // Controllers (button bindings are made in the OpModes, e.g. DriverTeleop)
        driver = new CommandGamepad(RobotMap.DRIVER_CONTROLLER);

        // Dashboard buttons that tell the robot where it was placed (Seed Left /
        // Center / Right), so autos know which side they start on without QuestNav.
        PoseSeeds.bind(swerve);

        // Give auto routines access to the robot and all its subsystems. Last, so
        // everything above exists.
        AutoBuilder.init(this);
    }
}
