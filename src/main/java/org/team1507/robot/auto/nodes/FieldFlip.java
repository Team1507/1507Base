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

import org.team1507.lib.core.util.Alliance;

/**
 * Field coordinate flip utilities.
 *
 * <p>All robot nodes in {@link Nodes} are defined in <b>Blue-origin coordinates</b>:
 * (0, 0) at the Blue driver station corner, X increasing toward Red, Y increasing
 * to the left from the driver's perspective.
 *
 * <p>Check the game manual each season for field symmetry type:
 * <ul>
 *   <li><b>Rotational symmetry</b> (e.g. Rapid React 2022, Reefscape 2025, Rebuilt 2026):
 *       Red = Blue rotated 180° around field center.
 *       X_red = LENGTH - X_blue, Y_red = WIDTH - Y_blue, heading_red = heading_blue + 180°.
 *   <li><b>Mirror symmetry</b> (e.g. Crescendo 2024):
 *       Red = Blue mirrored across center line (X only inverted).
 *       Update the formulas below if your season uses mirror symmetry.
 * </ul>
 *
 * <p>These methods are called automatically by {@link org.team1507.robot.auto.AutoSequence}
 * when {@link Alliance#isRed()} returns true. Write all coordinates in Blue-origin and
 * the flip is applied at runtime — auto routines never call FieldFlip directly.
 *
 * @see Alliance#isRed()
 * @see Nodes.Field#LENGTH
 * @see Nodes.Field#WIDTH
 */
public final class FieldFlip {

    private FieldFlip() {}

    /**
     * Converts a robot pose from Blue-origin to Red-alliance coordinates.
     *
     * @param blue pose defined in Blue-origin field coordinates
     * @return equivalent pose in Red-alliance field coordinates
     */
    public static Pose2d pose(Pose2d blue) {
        return new Pose2d(
            translation(blue.getTranslation()),
            rotation(blue.getRotation())
        );
    }

    /**
     * Converts a field position from Blue-origin to Red-alliance coordinates.
     *
     * @param blue position defined in Blue-origin field coordinates
     * @return equivalent position in Red-alliance field coordinates
     */
    public static Translation2d translation(Translation2d blue) {
        return new Translation2d(
            Nodes.Field.LENGTH - blue.getX(),
            Nodes.Field.WIDTH  - blue.getY()
        );
    }

    /**
     * Converts a heading from Blue-origin to Red-alliance direction.
     * Adds 180° — update this formula if the season uses mirror symmetry.
     *
     * @param blue heading defined relative to Blue-origin field frame
     * @return equivalent heading in Red-alliance field frame
     */
    public static Rotation2d rotation(Rotation2d blue) {
        return blue.rotateBy(Rotation2d.fromDegrees(180.0));
    }
}