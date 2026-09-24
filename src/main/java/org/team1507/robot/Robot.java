//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot;

import org.wpilib.command2.button.CommandGamepad;

import org.team1507.lib.core.framework.LoggedRobot;
import org.team1507.robot.auto.AutoBuilder;
import org.team1507.robot.Constants.RobotMap;
import org.team1507.robot.subsystems.*;

// ─────────────────────────────────────────────────────────────────────────────
// Robot
//
// Holds everything shared by all OpModes: subsystems, controllers, and the
// driver button bindings. What the robot DOES in each mode lives in OpMode
// classes, which WPILib finds automatically and lists in the Driver Station:
//   - Autonomous: robot/auto/routines/  (@Autonomous, extend AutoOpMode)
//   - Teleop:     robot/teleop/         (@Teleop)
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
        // then re-add the "Set Pose Left/Start/Right" dashboard commands
        // (questNav.setKnownPoseCommand(Nodes.Robot.Start.X).named(...).publishToDashboard()).

        // Give auto routines access to the subsystems
        AutoBuilder.init(swerve);

        // Controllers and bindings
        driver = new CommandGamepad(RobotMap.DRIVER_CONTROLLER);
        configureBindings();
    }

    // =========================================================================
    // Bindings
    //
    // Bindings made here work in every mode. Keep them here, not in OpModes:
    // with Commands v2 a binding is never removed, so an OpMode that binds
    // buttons would add a duplicate each time it is selected.
    // =========================================================================

    private void configureBindings() {

        // ── Driver — swerve utilities ──────────────────────────────────────

        // Swerve brake — hold Start to lock wheels in X pattern, release to resume driving
        driver.start().whileTrue(swerve.brakeCommand());

        // Failsafe — cancels all running commands (see RobotBehaviors for details)
        driver.back().onTrue(RobotBehaviors.failsafe());

        // Point the robot toward the opposing alliance wall, then press the bottom
        // face button (A on Xbox) to zero the gyro. Do this after any hot code
        // deploy without a power cycle.
        driver.faceDown().onTrue(swerve.zeroHeadingCommand());
    }
}
