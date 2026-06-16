//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.util;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Alliance-color utilities for runtime alliance detection.
 *
 * <p>All coordinates in {@code Nodes.java} are stored in Blue-origin frame:
 * (0, 0) at the Blue driver station corner, X increasing toward the Red alliance
 * wall. Use {@link #isRed()} to determine whether those coordinates need to be
 * flipped before passing to Swerve or other field-frame logic.
 *
 * <p>When no FMS is connected (practice, simulation), {@link #isRed()} returns
 * {@code false} so that coordinates pass through unflipped and the robot behaves
 * as if it is on Blue alliance. To test Red-alliance behavior without a field,
 * set the alliance color in the DriverStation application before enabling.
 *
 * @see edu.wpi.first.wpilibj.DriverStation#getAlliance()
 */
public final class Alliance {

    private Alliance() {}

    /**
     * Returns {@code true} if the robot is currently assigned to the Red alliance.
     *
     * <p>When the alliance is unknown — for example, when no FMS is connected or
     * before the Driver Station has reported alliance assignment — this method
     * returns {@code false}, treating the robot as Blue. This matches WPILib
     * convention and means all Blue-origin coordinates pass through unmodified
     * during practice sessions.
     *
     * @return {@code true} if Red alliance; {@code false} if Blue or unknown
     */
    public static boolean isRed() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Red;
    }
}
