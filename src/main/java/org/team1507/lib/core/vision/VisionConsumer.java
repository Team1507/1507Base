//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Callback interface for feeding a vision-estimated pose into a drivetrain's
 * pose estimator.
 *
 * <p>Pass {@code swerve::addVisionMeasurement} as the consumer when constructing
 * a vision subsystem:
 * <pre>
 *   questNav = new QuestNavSubsystem(swerve::addVisionMeasurement, swerve::resetPose);
 * </pre>
 */
@FunctionalInterface
public interface VisionConsumer {

    /**
     * Accepts a vision-measured robot pose and adds it to the drivetrain's
     * pose estimator with a confidence weight.
     *
     * @param pose              estimated robot pose on the field (meters, radians)
     * @param timestampSeconds  FPGA timestamp when the measurement was captured
     * @param stdDevs           measurement confidence [x (m), y (m), heading (rad)]
     *                          — lower values = trust vision more
     */
    void addMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs);
}
