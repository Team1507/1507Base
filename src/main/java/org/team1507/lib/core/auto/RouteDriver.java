//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.auto;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Decides how the robot moves toward the route's current node: the part that
 * differs between the classic driver (straight lines) and the policy driver
 * (the trained network). Everything else (when a node counts as reached, time
 * cutoffs, giving up, turning, settling onto a node to wait for heading or a
 * hold, the final approach to an endpoint) is the route runner's job, so both
 * drivers behave the same way at every node.
 */
public interface RouteDriver {

    /**
     * Field-relative translation velocity (m/s) for this loop.
     *
     * @param pose       the robot's field position
     * @param fieldSpeed the robot's measured field velocity
     * @param current    the node the robot is driving to
     * @param next       the node after it in this part of the route (the same node at the end)
     */
    Translation2d translation(Pose2d pose, ChassisVelocities fieldSpeed,
                              RouteNode current, RouteNode next);
}
