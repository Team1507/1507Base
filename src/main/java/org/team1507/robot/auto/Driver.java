//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.auto;

/**
 * What drives an auto's route between nodes.
 *
 * <pre>
 *   new AutoSequence()                 // CLASSIC (the default)
 *   new AutoSequence(Driver.POLICY)    // the trained policy
 * </pre>
 *
 * Policy and classic autos are separate routines (e.g. "Sweep" and
 * "Sweep - Policy"): they use different nodes, because the classic driver needs
 * extra nodes to steer around obstacles and the policy avoids them itself. If
 * the policy acts strangely, pick the classic version on the Driver Station.
 */
public enum Driver {
    /** Straight lines between nodes (the 2026 moveThrough / driveTo behavior). */
    CLASSIC,
    /** The trained policy network from Swerve-Policy-Playground. Nodes at most 5 m apart. */
    POLICY
}
