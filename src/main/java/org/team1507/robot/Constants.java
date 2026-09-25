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

    // The power distribution hub (PDH). Its per-channel currents are logged
    // every loop, to trace brownouts to the mechanism that caused them. It must
    // keep its default CAN ID (1).
    // TODO(SEASON LOGGING-1): confirm which SystemCore port the PDH is wired to.
    public static final CANPort PDH_CAN_PORT = CANPort.CAN_S0;

    // ============================================================
    // Hardware Map — CAN IDs and ports for everything EXCEPT the
    // drivetrain (see below). Update whenever a device is replaced
    // or renumbered.
    // ============================================================

    public static final class RobotMap {

        // Drivetrain CAN IDs, CANcoder offsets, and the Pigeon2 ID live in the
        // Tuner X paste zone at the top of subsystems/SwerveConfig.java, so they can be
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
    // limits) live in subsystems/SwerveConfig.java. They only change when the
    // hardware changes, and they are pasted from Tuner X.
    // ============================================================

    public static final class kSwerve {

        // TODO(SEASON SWERVE-9): retune these once the robot drives (heading control,
        // auto arrival thresholds, pose-estimator trust, wheel-wear scale).
        public static final class kTuning {

            /** Joystick deadband for teleop driving: stick input below this counts as zero. */
            public static final double DRIVER_DEADBAND = 0.12;

            /** Teleop rotation speed at full right-stick (rad/s). Math.PI = half a turn per second. */
            public static final double DRIVER_MAX_ROTATION = Math.PI;

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

            /** Proportional gain for heading control (rad/s per radian of error). Used everywhere the robot turns. */
            public static final double HEADING_KP = 5.5;

            /**
             * Stuck detection for auto driving (routes and driveForwardMeters). If
             * the robot moves less than STALL_THRESHOLD meters over STALL_TIMEOUT
             * seconds, the drive gives up so the auto can continue instead of
             * pushing into a wall forever.
             */
            public static final double STALL_THRESHOLD = 0.02; // meters
            public static final double STALL_TIMEOUT   = 1.5;  // seconds

            /** How close counts as "arrived" at an endpoint, and for driveForwardMeters (meters). */
            public static final double ARRIVE_THRESHOLD = 0.05; // 5 cm

            /**
             * Slow-down gain on the way into an endpoint: speed = ARRIVE_KP × distance
             * (m/s per meter). Slowing starts at cruiseSpeed / ARRIVE_KP meters out:
             * at 5.04 m/s with ARRIVE_KP = 2.5, about 2.0 m from the endpoint.
             */
            public static final double ARRIVE_KP = 2.5;

            /**
             * Heading tolerance (degrees) for endpoints with .heading() / .facing(),
             * and for the turn-in-place commands (pointToTarget, changeHeading).
             */
            public static final double HEADING_TOLERANCE_DEG = 3.0;

            /** Turn-in-place commands give up after this long, so a pinned robot can't stall an auto (seconds). */
            public static final double MAX_TURN_SECONDS = 2.0;

            /**
             * Lead time for motion compensation in maintainHeadingToTarget (seconds).
             * Increase if shots lag while moving; decrease if they lead too far.
             */
            public static final double AIM_LEAD_TIME = 0.25;
        }
    }

    // ============================================================
    // Autonomous routes: how accurate checkpoints must be, and the
    // route runner's safety limits. See robot/auto/AutoSequence.java.
    // ============================================================

    public static final class kAuto {

        /**
         * How close a checkpoint (or waypoint) must be before it counts as
         * reached: distance AND heading. Use in routines as
         * {@code .checkpoint(node, Accuracy.TIGHT)}; NORMAL when left out.
         *
         * <p>Change a value here and every auto that uses it changes. Starting
         * values come from the pass radii the 2026 autos used (0.1–0.7 m).
         */
        // TODO(SEASON SWERVE-9): tune these on the robot along with the other driving knobs.
        public enum Accuracy {
            /** Lining up before a bump or a narrow gap. */
            PRECISE(0.10, 5.0),
            /** Most checkpoints. */
            TIGHT(0.20, 15.0),
            /** The default when no accuracy is given. */
            NORMAL(0.40, 30.0),
            /** Roughly there; fastest. */
            LOOSE(0.70, 45.0);

            /** How close to the node's position (meters). */
            public final double meters;
            /** How close to the node's heading (degrees). */
            public final double degrees;

            Accuracy(double meters, double degrees) {
                this.meters = meters;
                this.degrees = degrees;
            }
        }

        /**
         * Within this distance of an endpoint (meters), the classic slow-down
         * finishes the approach whichever driver is running, so endpoints are
         * always precise. (The policy was trained to pass nodes at 0.65 m.)
         */
        public static final double ENDPOINT_HANDOFF = 0.65;

        /** Give up on a node that takes longer than this (seconds). Catches circling. */
        public static final double MAX_SECONDS_PER_NODE = 5.0;

        /** How long the robot may sit on a node waiting for its heading before moving on (seconds). */
        public static final double HEADING_WAIT_SECONDS = 1.0;

        /**
         * How long the robot may sit on a node waiting for a .holdUntil(...)
         * condition before moving on anyway (seconds), so a jammed mechanism
         * can't stop the whole auto. Use .by(...) on the node for a tighter limit.
         */
        public static final double HOLD_TIMEOUT_SECONDS = 3.0;

        /** Policy routes: nodes must be at most this far apart (meters); the policy was trained on 1–6 m. */
        public static final double POLICY_MAX_NODE_SPACING = 5.0;
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
