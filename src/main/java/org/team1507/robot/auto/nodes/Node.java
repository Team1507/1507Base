//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.auto.nodes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

// ─────────────────────────────────────────────────────────────────────────────
// Node
//
// Two factory methods with intentionally different return types:
//
//   Node.at(x, y, degrees)  →  Pose2d       — robot navigation targets
//   Node.location(x, y)     →  Translation2d — field structure positions
//
// The type difference is the distinction: robot nodes have a heading because
// the robot must face a specific direction when it arrives. Field element nodes
// are just locations — they describe where a structure is, not how to face it.
// Passing a Translation2d to driveToPoint() is a compile error, which forces
// students to think about what heading the robot needs before using the position.
// ─────────────────────────────────────────────────────────────────────────────
public final class Node {

    private Node() {}

    /** Robot navigation target — position in meters, heading in degrees. */
    public static Pose2d at(double xMeters, double yMeters, double headingDegrees) {
        return new Pose2d(xMeters, yMeters, Rotation2d.fromDegrees(headingDegrees));
    }

    /** Field structure position — location only, no heading. */
    public static Translation2d location(double xMeters, double yMeters) {
        return new Translation2d(xMeters, yMeters);
    }
}
