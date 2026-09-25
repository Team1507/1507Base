//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.auto;

import java.util.function.BooleanSupplier;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * One point on an auto route, already placed on the field (Red flipping and
 * left/right mirroring applied). AutoSequence builds these; the route runner
 * drives through them.
 *
 * @param type                what the robot does at this node
 * @param target              where the node is (field coordinates)
 * @param heading             which way the robot turns toward while driving to it
 * @param distanceTolerance   how close counts as "reached" (meters)
 * @param headingToleranceDeg how close the heading must be when it is checked (degrees)
 * @param checksHeading       true if "reached" also requires the heading to be within tolerance
 * @param speed               top speed on the way to this node (m/s)
 * @param cutoffSeconds       auto time after which the node counts as reached anyway
 *                            ({@code Double.POSITIVE_INFINITY} for no cutoff)
 * @param holdUntil           the robot stays on the node until this is true ({@code .holdUntil});
 *                            null for no hold
 */
public record RouteNode(
    Type type,
    Pose2d target,
    Rotation2d heading,
    double distanceTolerance,
    double headingToleranceDeg,
    boolean checksHeading,
    double speed,
    double cutoffSeconds,
    BooleanSupplier holdUntil
) {

    /** What the robot does at a node. */
    public enum Type {
        /** Drive through it and keep going; steps after it run while driving. */
        CHECKPOINT,
        /** Only shapes the route; nothing waits for it. */
        WAYPOINT,
        /** Drive to it and stop. */
        ENDPOINT
    }
}
