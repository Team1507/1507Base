//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.auto;

import org.wpilib.command3.Mechanism;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * What the route runner needs from a drivetrain. The robot's Swerve implements
 * it, which keeps the auto library independent of any one year's drivetrain.
 */
public interface RouteDrivetrain extends Mechanism {

    /** The robot's field position (Blue-origin field coordinates). */
    Pose2d getPose();

    /** The robot's measured velocity in field directions. */
    ChassisVelocities getFieldRelativeSpeeds();

    /** Drives with a FIELD-relative velocity (vx, vy in m/s, omega in rad/s). */
    void driveFieldRelative(ChassisVelocities fieldVelocities);

    /** Stops driving. */
    void stop();

    /** Top turning rate (rad/s). */
    double getMaxAngular();
}
