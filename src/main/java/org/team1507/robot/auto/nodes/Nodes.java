//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.auto.nodes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

// ─────────────────────────────────────────────────────────────────────────────
// Nodes
//
// Named navigation poses organized by category. All coordinates are in meters,
// in WPILib field coordinates with (0, 0) at the Blue DS corner.
//
// Type convention — visible at a glance in any editor:
//   Pose2d       — robot nodes (Node.at);      the robot drives here with a heading
//   Translation2d — field nodes (Node.location); describes where a structure is
//
// Three categories:
//   Field        — physical boundary corners (Translation2d — locations only)
//   Robot        — where the robot is commanded to drive, organized by role:
//                    Start, Score, Pickup, Waypoint (all Pose2d)
//   FieldElements — where physical field structures are located (Translation2d);
//                  repopulate each season after game reveal
//
// Naming conventions:
//   - LEFT/RIGHT are always driver-relative (from driver's perspective looking
//     onto the field). Add a comment if you deviate from this.
//   - APPROACH nodes are 1+ m from their target, giving the APF controller
//     room to decelerate before the final position.
//   - STATION nodes are the final docked positions at a field structure.
//
// To add a robot node:   Node.at(x, y, degrees)  → Pose2d       → goes in Robot.*
// To add a field node:   Node.location(x, y)     → Translation2d → goes in Field or FieldElements.*
// ─────────────────────────────────────────────────────────────────────────────
public final class Nodes {

    private Nodes() {}

    // ─────────────────────────────────────────────────────────────────────
    // Robot — where the robot is commanded to drive (Pose2d)
    //
    // Organized by role. LEFT/RIGHT are driver-relative throughout.
    // Change a node here once and every auto that references it updates.
    // ─────────────────────────────────────────────────────────────────────
    public static final class Robot {

        // Starting positions — where the robot is placed before the match.
        public static final class Start {
            public static final Pose2d RIGHT  = Node.at(3.5, 2.5,  55.0);
            public static final Pose2d CENTER = Node.at(2.0, 4.1,   0.0);
            public static final Pose2d LEFT   = Node.at(3.5, 5.7, 305.0);
        }

        // Scoring poses — where the robot faces to score a game piece.
        public static final class Score {
            public static final Pose2d RIGHT = Node.at(3.3, 2.8, 49.52);
            public static final Pose2d LEFT  = Node.at(3.4, 5.3, 320.3);
        }

        // Pickup poses — APPROACH is 1 m out from the station wall,
        // STATION is the final docked position against the wall.
        public static final class Pickup {
            public static final Pose2d APPROACH_RIGHT = Node.at(0.9, 0.70, 180.0);
            public static final Pose2d STATION_RIGHT  = Node.at(0.5, 0.70, 180.0);
            public static final Pose2d APPROACH_LEFT  = Node.at(0.9, 7.12, 180.0);
            public static final Pose2d STATION_LEFT   = Node.at(0.5, 8.12, 180.0);
        }

        // Waypoints — intermediate navigation poses used when routing across
        // the field. Not tied to any specific game action.
        public static final class Waypoint {
            public static final Pose2d MIDFIELD_RIGHT  = Node.at(8.27, 2.0, 0.0);
            public static final Pose2d MIDFIELD_CENTER = Node.at(8.27, 4.1, 0.0);
            public static final Pose2d MIDFIELD_LEFT   = Node.at(8.27, 6.2, 0.0);
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Field — physical boundary dimensions and corners (Translation2d)
    //
    // No heading — corners are reference locations, not robot destinations.
    //
    // PRACTICE_MODE — set to true when working in the field room.
    // NodeBoundsTest reads this flag and validates all nodes against the
    // smaller practice dimensions. Any node outside those bounds fails the
    // build so the robot cannot be deployed with an out-of-bounds coordinate.
    //
    // Measure your field room and update PRACTICE_LENGTH / PRACTICE_WIDTH.
    // ─────────────────────────────────────────────────────────────────────
    public static final class Field {

        // Competition field — 2025/2026 standard dimensions
        public static final double LENGTH = 16.54; // meters
        public static final double WIDTH  =  8.21; // meters

        // Practice field room — update these to match your actual space
        public static final double PRACTICE_LENGTH = 8.27; // TODO: measure your field room
        public static final double PRACTICE_WIDTH  = 8.21; // TODO: measure your field room

        // ── Flip this to true when building for the practice field room ──
        public static final boolean PRACTICE_MODE = false;

        public static final Translation2d ORIGIN          = Node.location(  0.00,  0.00);
        public static final Translation2d BLUE_FAR_CORNER = Node.location(  0.00, WIDTH);
        public static final Translation2d RED_NEAR_CORNER = Node.location(LENGTH,  0.00);
        public static final Translation2d RED_FAR_CORNER  = Node.location(LENGTH, WIDTH);
    }

    // ─────────────────────────────────────────────────────────────────────
    // FieldElements — where physical field structures are located (Translation2d)
    //
    // One nested class per structure. Each class exposes:
    //   CORNER_* — named corners of the structure (Translation2d), for readability
    //   CORNERS  — ordered array of all corners, used by NodeBoundsTest.
    //              Walk edges as consecutive pairs (wrapping last→first).
    //              Works for any convex polygon — square, octagon, etc.
    //              No bounding box needed; the test derives geometry from CORNERS.
    //
    // Corner naming: NEAR = closer to blue DS (low X), FAR = far side (high X).
    //                LEFT/RIGHT are driver-relative (left = high Y in WPILib coords).
    // Winding order in CORNERS: clockwise when viewed from above (x right, y up).
    //
    // Repopulate each season after game reveal with measured field coordinates.
    // ─────────────────────────────────────────────────────────────────────
    public static final class FieldElements {

        public static final class Hub {
            // 47 in × 47 in box, centered on field Y, near edge at the Blue Alliance Zone.
            // Source: Team 340 field measurements — HUB_WIDTH = 47 in = 1.194 m,
            //         BLUE_ZONE = AprilTag 26 X ≈ 3.048 m, Y_CENTER = 8.21 / 2 = 4.105 m.
            public static final Translation2d CORNER_NEAR_LEFT  = Node.location(3.048, 4.702);
            public static final Translation2d CORNER_FAR_LEFT   = Node.location(4.242, 4.702);
            public static final Translation2d CORNER_FAR_RIGHT  = Node.location(4.242, 3.508);
            public static final Translation2d CORNER_NEAR_RIGHT = Node.location(3.048, 3.508);

            // Ordered clockwise from top-left. NodeBoundsTest walks these edges
            // to check if any robot node's spin circle intersects the structure.
            public static final Translation2d[] CORNERS = {
                CORNER_NEAR_LEFT,
                CORNER_FAR_LEFT,
                CORNER_FAR_RIGHT,
                CORNER_NEAR_RIGHT,
            };
        }

        // Add one class per field structure after game reveal:
        //   public static final class Trench { ... }
    }
}
