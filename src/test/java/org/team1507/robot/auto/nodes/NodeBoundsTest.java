//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.auto.nodes;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import org.junit.jupiter.api.Test;
import org.team1507.robot.Constants.kSwerve;

// ─────────────────────────────────────────────────────────────────────────────
// NodeBoundsTest
//
// Two build-time validations that run as part of `./gradlew build`:
//
//   allNodesWithinBounds      — every node is inside the active field boundary
//   noRobotNodeCollidesWithObstacles — no robot node places the robot inside
//                                      a field structure
//
// Both tests must pass before the build produces a deployable binary.
//
// FIELD BOUNDARY (allNodesWithinBounds):
//   Set Nodes.Field.PRACTICE_MODE = true when building at the field room.
//   The test validates against PRACTICE_LENGTH / PRACTICE_WIDTH instead.
//   Set it back to false before a competition build.
//
// COLLISION DETECTION (noRobotNodeCollidesWithObstacles):
//   Each robot node is modeled as a circle:
//     center = node XY position
//     radius = robot half-diagonal (center-to-module corner) + BUMPER_BUFFER
//   Each field element is modeled as a polygon defined by its CORNERS[] array.
//   Collision = the robot's circle overlaps the polygon, which is true when:
//     (a) the node center is inside the polygon, OR
//     (b) the distance from the node center to any polygon edge < robot radius.
//
//   The CORNERS[] array on each FieldElements class drives the check automatically.
//   Works for any convex polygon — square, octagon, etc. No bounding box needed.
//
//   To add a new obstacle each season, add one obstacle() call inside
//   clearanceCheck() for the new FieldElements class.
//
// ADDING NEW NODES:
//   Add a matching check() and clearanceCheck() call for each new constant.
//   Use the full Nodes path as the name string — it appears in failure messages.
// ─────────────────────────────────────────────────────────────────────────────
class NodeBoundsTest {

    // =========================================================================
    // Field boundary limits
    // =========================================================================

    private static final double MAX_X = Nodes.Field.PRACTICE_MODE
        ? Nodes.Field.PRACTICE_LENGTH
        : Nodes.Field.LENGTH;

    private static final double MAX_Y = Nodes.Field.PRACTICE_MODE
        ? Nodes.Field.PRACTICE_WIDTH
        : Nodes.Field.WIDTH;

    // =========================================================================
    // Robot footprint
    //
    // The robot is approximated as a circle for collision purposes.
    // Radius = distance from robot center to the outermost swerve module corner,
    // plus bumper thickness. This is conservative (a circle is larger than the
    // actual rectangular footprint), which is the safe direction to err.
    // =========================================================================

    /** FRC bumper thickness — added to the module half-diagonal for clearance. */
    private static final double BUMPER_BUFFER_METERS = 0.1;

    /**
     * Clearance radius of the robot.
     * FRONT_LEFT_LOCATION.getNorm() = distance from robot center to module corner.
     * This matches the radius used for MAX_ANGULAR_RATE in Constants.
     */
    private static final double ROBOT_CLEARANCE_RADIUS =
        kSwerve.FRONT_LEFT_LOCATION.getNorm() + BUMPER_BUFFER_METERS;


    // =========================================================================
    // Test 1 — Field boundary
    // =========================================================================

    @Test
    void allNodesWithinBounds() {

        // ── Robot.Start ───────────────────────────────────────────────────
        check("Robot.Start.RIGHT",  Nodes.Robot.Start.RIGHT);
        check("Robot.Start.CENTER", Nodes.Robot.Start.CENTER);
        check("Robot.Start.LEFT",   Nodes.Robot.Start.LEFT);

        // ── Robot.Score ───────────────────────────────────────────────────
        check("Robot.Score.RIGHT", Nodes.Robot.Score.RIGHT);
        check("Robot.Score.LEFT",  Nodes.Robot.Score.LEFT);

        // ── Robot.Pickup ──────────────────────────────────────────────────
        check("Robot.Pickup.APPROACH_RIGHT", Nodes.Robot.Pickup.APPROACH_RIGHT);
        check("Robot.Pickup.STATION_RIGHT",  Nodes.Robot.Pickup.STATION_RIGHT);
        check("Robot.Pickup.APPROACH_LEFT",  Nodes.Robot.Pickup.APPROACH_LEFT);
        check("Robot.Pickup.STATION_LEFT",   Nodes.Robot.Pickup.STATION_LEFT);

        // ── Robot.Waypoint ────────────────────────────────────────────────
        check("Robot.Waypoint.MIDFIELD_RIGHT",  Nodes.Robot.Waypoint.MIDFIELD_RIGHT);
        check("Robot.Waypoint.MIDFIELD_CENTER", Nodes.Robot.Waypoint.MIDFIELD_CENTER);
        check("Robot.Waypoint.MIDFIELD_LEFT",   Nodes.Robot.Waypoint.MIDFIELD_LEFT);

        // ── FieldElements ─────────────────────────────────────────────────
        // Add checks here when FieldElements is populated after game reveal.
    }


    // =========================================================================
    // Test 2 — Field element collision
    // =========================================================================

    @Test
    void noRobotNodeCollidesWithObstacles() {

        // ── Robot.Start ───────────────────────────────────────────────────
        clearanceCheck("Robot.Start.RIGHT",  Nodes.Robot.Start.RIGHT);
        clearanceCheck("Robot.Start.CENTER", Nodes.Robot.Start.CENTER);
        clearanceCheck("Robot.Start.LEFT",   Nodes.Robot.Start.LEFT);

        // ── Robot.Score ───────────────────────────────────────────────────
        clearanceCheck("Robot.Score.RIGHT", Nodes.Robot.Score.RIGHT);
        clearanceCheck("Robot.Score.LEFT",  Nodes.Robot.Score.LEFT);

        // ── Robot.Pickup ──────────────────────────────────────────────────
        clearanceCheck("Robot.Pickup.APPROACH_RIGHT", Nodes.Robot.Pickup.APPROACH_RIGHT);
        clearanceCheck("Robot.Pickup.STATION_RIGHT",  Nodes.Robot.Pickup.STATION_RIGHT);
        clearanceCheck("Robot.Pickup.APPROACH_LEFT",  Nodes.Robot.Pickup.APPROACH_LEFT);
        clearanceCheck("Robot.Pickup.STATION_LEFT",   Nodes.Robot.Pickup.STATION_LEFT);

        // ── Robot.Waypoint ────────────────────────────────────────────────
        clearanceCheck("Robot.Waypoint.MIDFIELD_RIGHT",  Nodes.Robot.Waypoint.MIDFIELD_RIGHT);
        clearanceCheck("Robot.Waypoint.MIDFIELD_CENTER", Nodes.Robot.Waypoint.MIDFIELD_CENTER);
        clearanceCheck("Robot.Waypoint.MIDFIELD_LEFT",   Nodes.Robot.Waypoint.MIDFIELD_LEFT);
    }

    /**
     * Checks a single robot node against every registered field element obstacle.
     * Add one obstacle() call here for each new FieldElements structure each season.
     */
    private void clearanceCheck(String name, Pose2d pose) {
        Translation2d position = pose.getTranslation();
        obstacle(name, position, "FieldElements.Hub", Nodes.FieldElements.Hub.CORNERS);

        // Add more obstacles here after game reveal:
        // obstacle(name, position, "FieldElements.Trench", Nodes.FieldElements.Trench.CORNERS);
    }


    // =========================================================================
    // Helpers
    // =========================================================================

    /** Boundary check for a robot Pose2d node. */
    private void check(String name, Pose2d pose) {
        check(name, pose.getX(), pose.getY());
    }

    /** Boundary check for a Translation2d field location. */
    private void check(String name, Translation2d point) {
        check(name, point.getX(), point.getY());
    }

    private void check(String name, double x, double y) {
        String bounds = String.format("%.2f m × %.2f m", MAX_X, MAX_Y);
        assertTrue(x >= 0,    name + " — x=" + x + " is negative (outside field)");
        assertTrue(x <= MAX_X, name + " — x=" + x + " exceeds field length " + MAX_X + " (" + bounds + ")");
        assertTrue(y >= 0,    name + " — y=" + y + " is negative (outside field)");
        assertTrue(y <= MAX_Y, name + " — y=" + y + " exceeds field width "  + MAX_Y + " (" + bounds + ")");
    }

    /**
     * Circle-polygon collision check between a robot node and a field obstacle.
     *
     * The robot is modeled as a circle centered at the node position with radius
     * ROBOT_CLEARANCE_RADIUS. The obstacle is a convex polygon defined by its corners.
     *
     * Two failure conditions:
     *   (a) node center is inside the polygon — the robot would be occupying the structure
     *   (b) distance from node center to any polygon edge < ROBOT_CLEARANCE_RADIUS —
     *       the robot's spin circle overlaps the structure
     *
     * Works for any polygon shape — square, octagon, etc. Add the corners in CORNERS[]
     * and the test handles the rest.
     *
     * @param nodeName     display name of the robot node (shown in failure message)
     * @param nodePos      XY position of the robot node
     * @param obstacleName display name of the obstacle (shown in failure message)
     * @param corners      ordered polygon corners (clockwise or counter-clockwise)
     */
    private void obstacle(
            String nodeName, Translation2d nodePos,
            String obstacleName, Translation2d[] corners) {

        // (a) Node is inside the polygon — definite collision regardless of radius
        if (isInsidePolygon(nodePos, corners)) {
            fail(String.format(
                "%s is inside %s — node at (%.3f, %.3f) is within the structure boundary",
                nodeName, obstacleName, nodePos.getX(), nodePos.getY()
            ));
            return;
        }

        // (b) Robot's spin circle reaches a polygon edge
        double minDistance = Double.MAX_VALUE;
        int n = corners.length;
        for (int i = 0; i < n; i++) {
            double d = pointToSegmentDistance(nodePos, corners[i], corners[(i + 1) % n]);
            if (d < minDistance) minDistance = d;
        }

        assertTrue(minDistance >= ROBOT_CLEARANCE_RADIUS, String.format(
            "%s — robot spin circle (r=%.3f m) intersects %s: "
            + "closest edge distance=%.3f m",
            nodeName, ROBOT_CLEARANCE_RADIUS, obstacleName, minDistance
        ));
    }

    /**
     * Ray-casting point-in-polygon test. Works for any simple polygon regardless
     * of winding order. Returns true if {@code point} is strictly inside {@code polygon}.
     */
    private boolean isInsidePolygon(Translation2d point, Translation2d[] polygon) {
        boolean inside = false;
        int n = polygon.length;
        double px = point.getX(), py = point.getY();
        for (int i = 0, j = n - 1; i < n; j = i++) {
            double xi = polygon[i].getX(), yi = polygon[i].getY();
            double xj = polygon[j].getX(), yj = polygon[j].getY();
            if (((yi > py) != (yj > py)) &&
                (px < (xj - xi) * (py - yi) / (yj - yi) + xi)) {
                inside = !inside;
            }
        }
        return inside;
    }

    /**
     * Returns the shortest distance from point {@code p} to the line segment {@code (a, b)}.
     * Projects {@code p} onto the segment, clamping to [0, 1] so the result is always
     * the distance to a point on the segment (not its infinite extension).
     */
    private double pointToSegmentDistance(Translation2d p, Translation2d a, Translation2d b) {
        double ax = a.getX(), ay = a.getY();
        double bx = b.getX(), by = b.getY();
        double px = p.getX(), py = p.getY();

        double abx = bx - ax, aby = by - ay;
        double ab2 = abx * abx + aby * aby;

        if (ab2 == 0.0) return p.getDistance(a); // degenerate segment (a == b)

        double t = ((px - ax) * abx + (py - ay) * aby) / ab2;
        t = Math.max(0.0, Math.min(1.0, t));

        return p.getDistance(new Translation2d(ax + t * abx, ay + t * aby));
    }
}
