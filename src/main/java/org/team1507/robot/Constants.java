//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot;

import com.ctre.phoenix6.CANBus;
import org.wpilib.hardware.bus.CANPort;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;

public class Constants {

    // ============================================================
    // CAN Bus — SystemCore has five CAN ports (CAN_S0 .. CAN_S4).
    // Every CTRE device must be told which bus it is on.
    // CAN_S0 replaces the roboRIO's single built-in "rio" bus.
    // If a CANivore is used, create it with new CANBus("canivore")
    // and pass that bus to the devices wired to it.
    // ============================================================

    // TODO(SEASON SWERVE-2): confirm which SystemCore port the drivetrain is wired to.
    public static final CANBus CAN_BUS = new CANBus(CANPort.CAN_S0);

    // ============================================================
    // Hardware Map — all CAN IDs and sensor offsets in one place.
    // Update whenever a motor or encoder is replaced or renumbered.
    // ============================================================

    public static final class RobotMap {

        // Drivetrain CAN IDs, CANcoder offsets, and the Pigeon2 ID live in the
        // Tuner X paste zone at the top of subsystems/Swerve.java, so they can be
        // pasted straight from Tuner X's generated TunerConstants.java.
        // Keep every OTHER device's CAN ID here, and check for collisions with
        // the drivetrain IDs (1-12, Pigeon2 = 30) when adding one.

        // Driver Station USB ports
        public static final int DRIVER_CONTROLLER   = 0;
        public static final int OPERATOR_CONTROLLER = 1;
    }

    // ============================================================
    // Swerve Drive tuning: knobs adjusted while tuning or at events.
    //
    // Swerve HARDWARE facts (gear ratios, wheel size, motor gains, current
    // limits) live in subsystems/Swerve.java. They only change when the
    // hardware changes, and they are pasted from Tuner X.
    // ============================================================

    public static final class kSwerve {

        // TODO(SEASON SWERVE-9): retune these once the robot drives (heading control,
        // auto arrival thresholds, pose-estimator trust, wheel-wear scale).
        public static final class kTuning {

            /**
             * Pose estimator standard deviations [x (m), y (m), heading (rad)].
             * Lower = trust that source more.
             */
            public static final Matrix<N3, N1> ODOMETRY_STD_DEV = VecBuilder.fill(0.02, 0.02, 0.05);
            public static final Matrix<N3, N1> VISION_STD_DEV   = VecBuilder.fill(0.02, 0.02, 0.05);

            /**
             * Scale factor applied to measured drive distance and speed. Use to
             * correct for worn wheels (1.0 = no correction). Do not set to zero.
             */
            public static final double DRIVE_METERS_SCALE = 1.0;

            /** Proportional gain for simple heading control (rad/s per radian of error). */
            public static final double HEADING_KP = 5.5;

            /** PID gains for the rotation controller in moveThroughPose. */
            public static final double THETA_KP = 4.0;
            public static final double THETA_KI = 0.0;
            public static final double THETA_KD = 0.1;

            /**
             * Default pass radius for moveThroughPose (meters).
             * How close the robot must get before the waypoint is considered passed.
             */
            public static final double MOVE_THROUGH_DEFAULT_RADIUS = 0.3;

            /**
             * Stall detection for moveThroughPose.
             * If the robot moves less than STALL_THRESHOLD meters over STALL_TIMEOUT seconds,
             * the command exits to prevent a permanent block.
             */
            public static final double STALL_THRESHOLD = 0.02; // meters
            public static final double STALL_TIMEOUT   = 1.5;  // seconds

            /** Wall-clock deadline for moveThroughPose: exits if the command runs longer than this. */
            public static final double MAX_MOVETHROUGH_SECONDS = 5.0;

            /** Arrival threshold for driveToPoint / driveForwardMeters (meters). */
            public static final double ARRIVE_THRESHOLD = 0.05; // 5 cm

            /**
             * P gain for the APF deceleration ramp in driveToPoint / driveForwardMeters.
             * Deceleration begins at cruiseVelocity / ARRIVE_KP meters from the target.
             * At MAX_SPEED (5.04 m/s): ARRIVE_KP=2.5 → decel starts ~2.0 m out.
             */
            public static final double ARRIVE_KP = 2.5;

            /** Finish angle tolerance for pointToTarget / changeHeading (degrees). */
            public static final double HEADING_TOLERANCE_DEG = 3.0;

            /**
             * Lead time for motion compensation in maintainHeadingToTarget (seconds).
             * Increase if shots lag while moving; decrease if they lead too far.
             */
            public static final double AIM_LEAD_TIME = 0.25;
        }
    }

    // ============================================================
    // QuestNav — Meta Quest headset vision configuration.
    // ============================================================

    public static final class kQuest {

        /**
         * Transform from robot center to the Quest's physical mounting point.
         *
         * <p>The Quest reports its own 3D pose at the headset, not at the robot
         * center. This offset is applied to convert between the two:
         * <ul>
         *   <li>Robot center → Quest mount: apply ROBOT_TO_QUEST (forward)</li>
         *   <li>Quest mount → Robot center: apply ROBOT_TO_QUEST.inverse()</li>
         * </ul>
         *
         * <p>Measured from the 2026 field season. Re-measure if the mount changes.
         * x = -0.202 m (behind center), y = 0.304 m (left of center),
         * z = 0.39 m (height), yaw = 90° (headset faces robot-left).
         */
        public static final Transform3d ROBOT_TO_QUEST = new Transform3d(
            -0.202, 0.304, 0.39,
            new Rotation3d(0, 0, Math.toRadians(90))
        );
    }
}
