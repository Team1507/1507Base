//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.auto;

import org.wpilib.math.geometry.Translation2d;

/**
 * Geometry for checking auto routes before they run: how close a straight leg
 * of a classic route passes to a field element (a convex polygon of corners,
 * like {@code Nodes.FieldElements.Hub.CORNERS}).
 */
public final class RouteGeometry {

    private RouteGeometry() {}

    /**
     * Smallest distance (m) between the straight leg a→b and the polygon.
     * 0 if the leg crosses the polygon or starts or ends inside it.
     */
    public static double clearance(Translation2d a, Translation2d b, Translation2d[] polygon) {
        if (contains(polygon, a) || contains(polygon, b)) {
            return 0.0;
        }
        double min = Double.POSITIVE_INFINITY;
        for (int i = 0; i < polygon.length; i++) {
            Translation2d p = polygon[i];
            Translation2d q = polygon[(i + 1) % polygon.length];
            if (segmentsCross(a, b, p, q)) {
                return 0.0;
            }
            min = Math.min(min, pointToSegment(p, a, b));
            min = Math.min(min, pointToSegment(a, p, q));
            min = Math.min(min, pointToSegment(b, p, q));
        }
        return min;
    }

    /** True if the point is inside the convex polygon (either winding order). */
    static boolean contains(Translation2d[] polygon, Translation2d point) {
        boolean anyPositive = false;
        boolean anyNegative = false;
        for (int i = 0; i < polygon.length; i++) {
            double c = cross(polygon[i], polygon[(i + 1) % polygon.length], point);
            anyPositive |= c > 0;
            anyNegative |= c < 0;
        }
        return !(anyPositive && anyNegative);
    }

    static double pointToSegment(Translation2d point, Translation2d a, Translation2d b) {
        double dx = b.getX() - a.getX();
        double dy = b.getY() - a.getY();
        double lengthSquared = dx * dx + dy * dy;
        if (lengthSquared < 1e-12) {
            return point.getDistance(a);
        }
        double t = ((point.getX() - a.getX()) * dx + (point.getY() - a.getY()) * dy) / lengthSquared;
        t = Math.max(0.0, Math.min(1.0, t));
        return point.getDistance(new Translation2d(a.getX() + t * dx, a.getY() + t * dy));
    }

    private static boolean segmentsCross(Translation2d a, Translation2d b, Translation2d p, Translation2d q) {
        double d1 = cross(a, b, p);
        double d2 = cross(a, b, q);
        double d3 = cross(p, q, a);
        double d4 = cross(p, q, b);
        return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0));
    }

    /** Which side of line o→a the point b is on (positive = left). */
    private static double cross(Translation2d o, Translation2d a, Translation2d b) {
        return (a.getX() - o.getX()) * (b.getY() - o.getY()) - (a.getY() - o.getY()) * (b.getX() - o.getX());
    }
}
