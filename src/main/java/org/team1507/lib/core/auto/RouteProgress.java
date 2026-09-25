//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.auto;

/**
 * How far along an auto route the robot is. Node indices count across the
 * whole route, in the order the routine lists them.
 *
 * <p>Only moves forward. A node counts as done when the robot reached it, when
 * its time cutoff passed, or when the route gave up or was canceled (so an auto
 * never waits forever for a node the robot will not reach).
 */
public final class RouteProgress {

    /** Nodes 0 .. done-1 are done. */
    private int done = 0;

    /** True once the robot is past this node (reached, cut off, or given up). */
    public boolean isDone(int index) {
        return index < done;
    }

    /** Marks every node up to and including {@code index} as done. */
    void markDoneThrough(int index) {
        done = Math.max(done, index + 1);
    }
}
