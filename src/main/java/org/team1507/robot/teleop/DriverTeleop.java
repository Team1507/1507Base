//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.teleop;

import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.MathUtil;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

import org.team1507.robot.Robot;
import org.team1507.robot.RobotBehaviors;
import org.team1507.robot.subsystems.SwerveConfig;

import static org.team1507.robot.Constants.kSwerve.kTuning.DRIVER_DEADBAND;
import static org.team1507.robot.Constants.kSwerve.kTuning.DRIVER_MAX_ROTATION;

// ─────────────────────────────────────────────────────────────────────────────
// DriverTeleop
//
// The match teleop mode (2027 OpMode framework, Commands v3). The Driver
// Station lists every @Teleop class in its teleop drop-down.
//
// Everything is set up in the constructor, which WPILib runs when this mode is
// SELECTED on the Driver Station. Commands v3 ties the default command and the
// button bindings made here to this OpMode: they work while it is selected
// (enabled or disabled) and are removed automatically when another OpMode is
// selected. There is nothing to clean up.
// ─────────────────────────────────────────────────────────────────────────────
@Teleop(name = "Driver")
public final class DriverTeleop implements OpMode {

    /** WPILib passes in the Robot instance when this mode is selected. */
    public DriverTeleop(Robot robot) {

        // ── Driving ────────────────────────────────────────────────────────

        // Joystick driving replaces swerve's idle default while this mode is selected.
        robot.swerve.setDefaultCommand(
            robot.swerve.driveCommand(() -> {
                double x   = MathUtil.applyDeadband(-robot.driver.getLeftY(),  DRIVER_DEADBAND);
                double y   = MathUtil.applyDeadband(-robot.driver.getLeftX(),  DRIVER_DEADBAND);
                double rot = MathUtil.applyDeadband(-robot.driver.getRightX(), DRIVER_DEADBAND);

                // Sticks are driver-relative: pushing forward always moves the robot
                // away from the driver, on either alliance. Convert to robot-relative
                // for the kinematics.
                return new ChassisVelocities(
                    x   * SwerveConfig.MAX_SPEED,
                    y   * SwerveConfig.MAX_SPEED,
                    rot * DRIVER_MAX_ROTATION
                ).toRobotRelative(robot.swerve.getDriverRelativeHeading());
            })
        );

        // ── Driver — swerve utilities ──────────────────────────────────────

        // Swerve brake — hold Start to lock wheels in X pattern, release to resume driving
        robot.driver.start().whileTrue(robot.swerve.brakeCommand());

        // Failsafe — interrupts every subsystem command (see RobotBehaviors for details)
        robot.driver.back().onTrue(RobotBehaviors.failsafe());

        // Point the robot straight away from the driver, then press the bottom face
        // button (A on Xbox) to reset its heading. Works while disabled, too. With
        // QuestNav/AprilTag vision this is only a fallback.
        robot.driver.faceDown().onTrue(robot.swerve.zeroHeadingCommand());
    }
}
