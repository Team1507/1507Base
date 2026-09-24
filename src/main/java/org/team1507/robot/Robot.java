//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.button.CommandXboxController;

import org.team1507.lib.core.framework.LoggedRobot;
import org.team1507.robot.auto.AutoBuilder;
import org.team1507.robot.auto.routines.*;
import org.team1507.robot.Constants.RobotMap;
import org.team1507.robot.Constants.kSwerve;
import org.team1507.robot.subsystems.*;

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

    private final CommandXboxController driver;

    // -------------------------------------------------------------------------
    // Autonomous
    // -------------------------------------------------------------------------

    private Command m_autoCommand = null;
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

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

        // Autonomous chooser
        AutoBuilder.init(swerve);
        autoChooser.setDefaultOption("Drive Forward", DriveForwardAuto.build());
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Controllers and bindings
        driver = new CommandXboxController(RobotMap.DRIVER_CONTROLLER);
        configureBindings();
        configureDefaultBindings();
    }

    // =========================================================================
    // Bindings
    // =========================================================================

    private void configureBindings() {

        // ── Driver — swerve utilities ──────────────────────────────────────

        // Swerve brake — hold Start to lock wheels in X pattern, release to resume driving
        driver.start().whileTrue(swerve.brakeCommand());

        // Failsafe — cancels all running commands (see RobotBehaviors for details)
        driver.back().onTrue(RobotBehaviors.failsafe());

        // Point the robot toward the opposing alliance wall, then press A
        // to zero the gyro. Do this after any hot code deploy without a power cycle.
        driver.a().onTrue(swerve.zeroHeadingCommand());
    }

    private void configureDefaultBindings() {

        swerve.setDefaultCommand(
            swerve.driveCommand(() -> {
                double x   = MathUtil.applyDeadband(-driver.getLeftY(),  0.12);
                double y   = MathUtil.applyDeadband(-driver.getLeftX(),  0.12);
                double rot = MathUtil.applyDeadband(-driver.getRightX(), 0.12);

                return ChassisVelocities.fromFieldRelativeSpeeds(
                    x   * kSwerve.MAX_SPEED,
                    y   * kSwerve.MAX_SPEED,
                    rot * Math.PI,
                    swerve.getHeading()
                );
            })
        );
    }

    // =========================================================================
    // Mode callbacks
    // =========================================================================

    @Override
    public void autonomousInit() {
        m_autoCommand = autoChooser.getSelected();
        if (m_autoCommand != null) {
            CommandScheduler.getInstance().schedule(m_autoCommand);
        }
    }

    @Override
    public void teleopInit() {
        if (m_autoCommand != null) {
            m_autoCommand.cancel();
        }
    }
}
