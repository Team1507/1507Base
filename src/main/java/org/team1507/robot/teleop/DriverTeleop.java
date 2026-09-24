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
import org.team1507.robot.subsystems.Swerve;

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
                double x   = MathUtil.applyDeadband(-robot.driver.getLeftY(),  0.12);
                double y   = MathUtil.applyDeadband(-robot.driver.getLeftX(),  0.12);
                double rot = MathUtil.applyDeadband(-robot.driver.getRightX(), 0.12);

                // Sticks are field-relative; convert to robot-relative for the kinematics.
                return new ChassisVelocities(
                    x   * Swerve.MAX_SPEED,
                    y   * Swerve.MAX_SPEED,
                    rot * Math.PI
                ).toRobotRelative(robot.swerve.getHeading());
            })
        );

        // ── Driver — swerve utilities ──────────────────────────────────────

        // Swerve brake — hold Start to lock wheels in X pattern, release to resume driving
        robot.driver.start().whileTrue(robot.swerve.brakeCommand());

        // Failsafe — interrupts every subsystem command (see RobotBehaviors for details)
        robot.driver.back().onTrue(RobotBehaviors.failsafe());

        // Point the robot toward the opposing alliance wall, then press the bottom
        // face button (A on Xbox) to zero the gyro. Works while disabled, too.
        robot.driver.faceDown().onTrue(robot.swerve.zeroHeadingCommand());
    }
}
