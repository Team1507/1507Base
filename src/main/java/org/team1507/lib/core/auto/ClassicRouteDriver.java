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
 * The classic driver: a straight line to each node. The same behavior as the
 * 2026 {@code moveThrough} (checkpoints and waypoints: constant speed, no
 * slowing down) and {@code driveTo} (endpoints: slow down on the way in).
 *
 * <p>It does not steer around anything. A classic route needs a node on each
 * side of an obstacle; the build-time route check flags legs that pass too
 * close to a field element.
 */
public final class ClassicRouteDriver implements RouteDriver {

    private final double arriveKp;

    /**
     * @param arriveKp slow-down gain for endpoints: speed = arriveKp × distance
     *                 (m/s per meter), capped at the node's speed
     */
    public ClassicRouteDriver(double arriveKp) {
        this.arriveKp = arriveKp;
    }

    @Override
    public Translation2d translation(Pose2d pose, ChassisVelocities fieldSpeed,
                                     RouteNode current, RouteNode next, boolean headingOk) {
        // Endpoints slow down on the way in. A checkpoint also slows down when the
        // robot is already there but still turning, so it settles on the node
        // instead of overshooting while the heading catches up.
        boolean slowDown = current.type() == RouteNode.Type.ENDPOINT || !headingOk;
        return toward(pose, current, slowDown ? arriveKp : Double.POSITIVE_INFINITY);
    }

    /**
     * Velocity straight toward {@code node}: the node's speed, or
     * {@code kp × distance} when that is smaller.
     */
    static Translation2d toward(Pose2d pose, RouteNode node, double kp) {
        Translation2d toTarget = node.target().getTranslation().minus(pose.getTranslation());
        double distance = toTarget.getNorm();
        if (distance < 1e-6) {
            return Translation2d.ZERO;
        }
        double speed = Math.min(node.speed(), kp * distance);
        return toTarget.times(speed / distance);
    }
}
