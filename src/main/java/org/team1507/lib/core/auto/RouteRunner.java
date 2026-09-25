//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.auto;

import java.util.List;
import java.util.function.DoubleSupplier;

import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.MathUtil;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;

/**
 * Drives one part of an auto route: from the node after the previous endpoint
 * up to and including the next endpoint (or the route's last node). AutoSequence
 * starts one of these in the background at each route's first path step.
 *
 * <p>Each loop, for the node the robot is driving to:
 * <ol>
 *   <li><b>Reached?</b> Within the node's distance tolerance, within its
 *       heading tolerance if it checks heading, and its {@code .holdUntil}
 *       condition (if any) is true. Then it is marked done and the robot heads
 *       for the next node without stopping.</li>
 *   <li><b>Time cutoff</b> ({@code .by(seconds)}): past it, the node counts as
 *       done anyway and the robot moves on.</li>
 *   <li><b>Waiting on the node</b> (in position, heading or hold not ready):
 *       the robot settles onto the node. The heading is waived after
 *       headingWaitSeconds; a hold gives up after holdTimeoutSeconds, and the
 *       robot moves on either way.</li>
 *   <li><b>Stuck</b> (not moving) or <b>too long</b> getting to one node: the
 *       route gives up, stops the robot, and marks the rest of this part done so
 *       the auto continues instead of waiting forever.</li>
 *   <li><b>Drive</b>: the driver (classic or policy) picks the direction and
 *       speed. Near an endpoint, and while settling onto a node, the classic
 *       slow-down takes over. The robot turns toward the node's heading the
 *       whole time.</li>
 * </ol>
 * At the end of the part the robot stops.
 *
 * <p>Logged under {@code Auto/Route/}: the target node's pose and index every
 * loop, and one line in {@code Auto/Route/Events} for every node reached, cut
 * off or given up.
 */
public final class RouteRunner {

    private static final String EVENTS = "Auto/Route/Events";
    private static final TelemetryTable TABLE = Telemetry.getTable("Auto/Route");

    static {
        Telemetry.keepDuplicates(EVENTS);
    }

    private RouteRunner() {}

    /**
     * The command that drives nodes {@code first} through {@code last} of the route.
     *
     * @param drivetrain the robot's drivetrain
     * @param nodes      the whole route (all parts), already placed on the field
     * @param first      index of the first node of this part
     * @param last       index of the last node of this part (an endpoint, or the route's last node)
     * @param driver     classic or policy
     * @param progress   shared progress; the auto's steps wait on it
     * @param config     tuning
     * @param autoTime   seconds since the auto started (for {@code .by} cutoffs)
     */
    public static Command drive(RouteDrivetrain drivetrain, List<RouteNode> nodes, int first, int last,
                                RouteDriver driver, RouteProgress progress, RouteConfig config,
                                DoubleSupplier autoTime) {
        return drivetrain.run(coroutine -> {
            int index = first;
            double nodeStart = Timer.getTimestamp();
            double headingWaitStart = Double.NaN;
            double holdStart = Double.NaN;
            boolean headingWaived = false;
            Translation2d progressPoint = drivetrain.getPose().getTranslation();
            double progressTime = nodeStart;

            while (index <= last) {
                RouteNode node = nodes.get(index);
                Pose2d pose = drivetrain.getPose();
                double now = Timer.getTimestamp();

                double distance = pose.getTranslation().getDistance(node.target().getTranslation());
                double headingErrorDeg = Math.abs(Math.toDegrees(MathUtil.angleModulus(
                    node.heading().minus(pose.getRotation()).getRadians())));
                boolean inPosition = distance <= node.distanceTolerance();
                boolean headingOk = headingWaived || !node.checksHeading()
                    || headingErrorDeg <= node.headingToleranceDeg();
                boolean holdOk = node.holdUntil() == null || node.holdUntil().getAsBoolean();

                // 1-3. Is this node done?
                String doneBecause = null;
                if (inPosition && headingOk && holdOk) {
                    doneBecause = "REACHED";
                } else if (autoTime.getAsDouble() >= node.cutoffSeconds()) {
                    doneBecause = "CUT OFF (time)";
                } else if (inPosition && !headingOk) {
                    // On the node, still turning.
                    if (Double.isNaN(headingWaitStart)) {
                        headingWaitStart = now;
                    } else if (now - headingWaitStart > config.headingWaitSeconds()) {
                        headingWaived = true;   // stop waiting for heading; a hold still applies
                        Telemetry.log(EVENTS, "HEADING WAIVED (timed out) node " + index);
                    }
                } else if (inPosition) {
                    // On the node, waiting for .holdUntil.
                    if (Double.isNaN(holdStart)) {
                        holdStart = now;
                    } else if (now - holdStart > config.holdTimeoutSeconds()) {
                        doneBecause = "REACHED (hold timed out)";
                    }
                } else {
                    headingWaitStart = Double.NaN;
                    holdStart = Double.NaN;
                }

                if (doneBecause != null) {
                    progress.markDoneThrough(index);
                    Telemetry.log(EVENTS, doneBecause + " node " + index + " " + node.type());
                    index++;
                    nodeStart = now;
                    headingWaitStart = Double.NaN;
                    holdStart = Double.NaN;
                    headingWaived = false;
                    progressPoint = pose.getTranslation();
                    progressTime = now;
                    continue;   // check the next node in the same loop
                }

                // 4. Stuck, or taking too long to get there? (Waiting ON the node
                // has its own time limits above.)
                if (pose.getTranslation().getDistance(progressPoint) > config.stallDistance()) {
                    progressPoint = pose.getTranslation();
                    progressTime = now;
                }
                boolean stuck = !inPosition && now - progressTime > config.stallSeconds();
                boolean tooLong = !inPosition && now - nodeStart > config.maxSecondsPerNode();
                if (stuck || tooLong) {
                    Telemetry.log(EVENTS, (stuck ? "GAVE UP (stuck)" : "GAVE UP (too long)")
                        + " at node " + index + "; skipping to node " + (last + 1));
                    break;
                }

                // 5. Drive. Settle onto the node (classic slow-down) near an endpoint,
                // or while waiting on a node for heading or a hold; otherwise the
                // driver decides.
                boolean settling = inPosition;   // in position but not done: waiting
                RouteNode next = nodes.get(Math.min(index + 1, last));
                ChassisVelocities fieldSpeed = drivetrain.getFieldRelativeSpeeds();
                Translation2d velocity =
                    settling || (node.type() == RouteNode.Type.ENDPOINT && distance < config.endpointHandoff())
                        ? ClassicRouteDriver.toward(pose, node, config.arriveKp())
                        : driver.translation(pose, fieldSpeed, node, next);

                double headingError = MathUtil.angleModulus(node.heading().minus(pose.getRotation()).getRadians());
                double omega = Math.clamp(headingError * config.headingKp(),
                    -drivetrain.getMaxAngular(), drivetrain.getMaxAngular());

                drivetrain.driveFieldRelative(new ChassisVelocities(velocity.getX(), velocity.getY(), omega));
                TABLE.log("Target", node.target());
                TABLE.log("Node", index);
                TABLE.log("Holding", inPosition && !holdOk);
                coroutine.yield();
            }

            drivetrain.stop();
            progress.markDoneThrough(last);
        })
        .whenCanceled(() -> {
            // Canceled (robot disabled, or another command took the drivetrain):
            // release every step waiting on this part so the auto can't hang.
            progress.markDoneThrough(last);
            drivetrain.stop();
            Telemetry.log(EVENTS, "CANCELED route to node " + last);
        })
        .named("Auto.route to node " + last);
    }
}
