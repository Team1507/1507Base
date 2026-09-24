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
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

import org.team1507.robot.Robot;
import org.team1507.robot.Constants.kSwerve;

// ─────────────────────────────────────────────────────────────────────────────
// DriverTeleop
//
// The match teleop mode (2027 OpMode framework). The Driver Station lists every
// @Teleop class in its teleop drop-down.
//
// start() gives the driver joystick control of swerve; end() takes it away again
// so nothing drives the robot outside teleop.
//
// Button bindings (brake, failsafe, zero heading) live in Robot.java, NOT here:
// with Commands v2 a binding is never removed, so creating bindings in an OpMode
// would add a duplicate every time the mode is selected again.
// ─────────────────────────────────────────────────────────────────────────────
@Teleop(name = "Driver")
public final class DriverTeleop extends PeriodicOpMode {

    private final Robot robot;

    /** WPILib passes in the Robot instance when this mode is selected. */
    public DriverTeleop(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void start() {
        robot.swerve.setDefaultCommand(
            robot.swerve.driveCommand(() -> {
                double x   = MathUtil.applyDeadband(-robot.driver.getLeftY(),  0.12);
                double y   = MathUtil.applyDeadband(-robot.driver.getLeftX(),  0.12);
                double rot = MathUtil.applyDeadband(-robot.driver.getRightX(), 0.12);

                // Sticks are field-relative; convert to robot-relative for the kinematics.
                return new ChassisVelocities(
                    x   * kSwerve.MAX_SPEED,
                    y   * kSwerve.MAX_SPEED,
                    rot * Math.PI
                ).toRobotRelative(robot.swerve.getHeading());
            })
        );
    }

    @Override
    public void end() {
        robot.swerve.removeDefaultCommand();
    }
}
