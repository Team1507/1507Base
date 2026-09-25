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
 *   <li><b>Reached?</b> Within the node's distance tolerance, and within its
 *       heading tolerance if it checks heading. Then it is marked done and the
 *       robot heads for the next node without stopping.</li>
 *   <li><b>Time cutoff</b> ({@code .by(seconds)}): past it, the node counts as
 *       done anyway and the robot moves on.</li>
 *   <li><b>Waiting for heading</b> too long on top of the node: counts as done.</li>
 *   <li><b>Stuck</b> (not moving) or <b>too long</b> on one node: the route
 *       gives up, stops the robot, and marks the rest of this part done so the
 *       auto continues instead of waiting forever.</li>
 *   <li><b>Drive</b>: the driver (classic or policy) picks the direction and
 *       speed; near an endpoint the classic slow-down takes over for accuracy.
 *       The robot turns toward the node's heading the whole time.</li>
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
            Translation2d progressPoint = drivetrain.getPose().getTranslation();
            double progressTime = nodeStart;

            while (index <= last) {
                RouteNode node = nodes.get(index);
                Pose2d pose = drivetrain.getPose();
                double now = Timer.getTimestamp();

                double distance = pose.getTranslation().getDistance(node.target().getTranslation());
                double headingErrorDeg = Math.abs(Math.toDegrees(MathUtil.angleModulus(
                    node.heading().minus(pose.getRotation()).getRadians())));
                boolean headingOk = !node.checksHeading() || headingErrorDeg <= node.headingToleranceDeg();
                boolean inPosition = distance <= node.distanceTolerance();

                // 1-3. Is this node done?
                String doneBecause = null;
                if (inPosition && headingOk) {
                    doneBecause = "REACHED";
                } else if (autoTime.getAsDouble() >= node.cutoffSeconds()) {
                    doneBecause = "CUT OFF (time)";
                } else if (inPosition) {
                    // In position, still turning.
                    if (Double.isNaN(headingWaitStart)) {
                        headingWaitStart = now;
                    } else if (now - headingWaitStart > config.headingWaitSeconds()) {
                        doneBecause = "REACHED (heading wait timed out)";
                    }
                } else {
                    headingWaitStart = Double.NaN;
                }

                if (doneBecause != null) {
                    progress.markDoneThrough(index);
                    Telemetry.log(EVENTS, doneBecause + " node " + index + " " + node.type());
                    index++;
                    nodeStart = now;
                    headingWaitStart = Double.NaN;
                    progressPoint = pose.getTranslation();
                    progressTime = now;
                    continue;   // check the next node in the same loop
                }

                // 4. Stuck, or taking too long?
                if (pose.getTranslation().getDistance(progressPoint) > config.stallDistance()) {
                    progressPoint = pose.getTranslation();
                    progressTime = now;
                }
                boolean stuck = !inPosition && now - progressTime > config.stallSeconds();
                boolean tooLong = now - nodeStart > config.maxSecondsPerNode();
                if (stuck || tooLong) {
                    Telemetry.log(EVENTS, (stuck ? "GAVE UP (stuck)" : "GAVE UP (too long)")
                        + " at node " + index + "; skipping to node " + (last + 1));
                    break;
                }

                // 5. Drive.
                RouteNode next = nodes.get(Math.min(index + 1, last));
                ChassisVelocities fieldSpeed = drivetrain.getFieldRelativeSpeeds();
                Translation2d velocity =
                    node.type() == RouteNode.Type.ENDPOINT && distance < config.endpointHandoff()
                        ? ClassicRouteDriver.toward(pose, node, config.arriveKp())
                        : driver.translation(pose, fieldSpeed, node, next, headingOk);

                double headingError = MathUtil.angleModulus(node.heading().minus(pose.getRotation()).getRadians());
                double omega = Math.clamp(headingError * config.headingKp(),
                    -drivetrain.getMaxAngular(), drivetrain.getMaxAngular());

                drivetrain.driveFieldRelative(new ChassisVelocities(velocity.getX(), velocity.getY(), omega));
                TABLE.log("Target", node.target());
                TABLE.log("Node", index);
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
