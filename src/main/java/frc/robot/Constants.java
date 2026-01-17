package frc.robot;


import static edu.wpi.first.units.Units.*;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

import frc.robot.generated.TunerConstants;

public class Constants {
    // ============================================================
    //  SPEED PROFILES
    //  Defines how fast the robot should move in different contexts.
    //  - MATCH:   Real‑world competition speed (safe + realistic)
    //  - SIM:     High-speed simulation mode (idealized physics)
    // ============================================================
    public enum SpeedProfile {
        MATCH,
        SIM
    }

    public static final class Speed {

        // ------------------------------------------------------------
        // TELEOP SCALING (Driver Controls)
        // ------------------------------------------------------------
        // These values scale the joystick inputs before they are sent
        // to the drivetrain during teleop. They ONLY affect manual
        // driving and have no impact on autonomous commands.
        //
        // Why scale teleop input?
        //   - Prevents the robot from moving too fast during indoor
        //     testing or tight‑space debugging.
        //   - Allows full-speed control during matches.
        //   - Lets simulation run at high speed without risk.
        //
        // Translation Scale:
        //   - Controls forward/strafe speed from the left joystick.
        //   - Real Robot: kept low for safety (0.15).
        //   - Simulation: higher for convenience (0.9).
        //
        // Rotation Scale:
        //   - Controls rotational speed from the right joystick.
        //   - Real Robot: reduced for smoother control (0.25).
        //   - Simulation: higher for fast testing (0.9).
        // ------------------------------------------------------------

        // Teleop translation scaling (driver control)
        public static double getTranslationScale() {
            return edu.wpi.first.wpilibj.RobotBase.isSimulation()
                ? 1.5    // fast, convenient movement in simulation
                : 0.15;  // safe, controlled speed on the real robot
        }

        // Teleop rotation scaling (driver control)
        public static double getRotationScale() {
            return edu.wpi.first.wpilibj.RobotBase.isSimulation()
                ? 1.5    // fast rotation in simulation
                : 0.25;  // smoother, safer rotation on real hardware
        }

        // ------------------------------------------------------------
        // SPEED PROFILE SELECTION
        // ------------------------------------------------------------
        // Automatically selects the appropriate speed profile:
        //   - SIM when running in WPILib simulation
        //   - MATCH on the real robot
        // ------------------------------------------------------------
        public static final SpeedProfile PROFILE =
            edu.wpi.first.wpilibj.RobotBase.isSimulation()
                ? SpeedProfile.SIM
                : SpeedProfile.MATCH;

        // ------------------------------------------------------------
        // MAX LINEAR SPEED
        // ------------------------------------------------------------
        // MATCH_MAX_SPEED:
        //   - The real robot’s physical top speed.
        //
        // SIM_MAX_SPEED:
        //   - A higher speed used only in simulation.
        //   - Allows fast, idealized motion without real-world limits.
        // ------------------------------------------------------------
        public static final double MATCH_MAX_SPEED = 
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

        public static final double SIM_MAX_SPEED = 8.0;

        // Returns the active max speed based on the current profile.
        public static double getMaxSpeed() {
            return switch (PROFILE) {
                case MATCH -> MATCH_MAX_SPEED;
                case SIM   -> SIM_MAX_SPEED;
            };
        }

        // ------------------------------------------------------------
        // MAX ANGULAR SPEED
        // ------------------------------------------------------------
        // MATCH_MAX_ANGULAR:
        //   - Rotational speed for competition.
        //
        // SIM_MAX_ANGULAR:
        //   - Very high rotational speed for simulation only.
        // ------------------------------------------------------------
        public static final double MATCH_MAX_ANGULAR =
            RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        public static final double SIM_MAX_ANGULAR = Math.toRadians(720);

        // Returns the active angular speed based on the current profile.
        public static double getMaxAngularSpeed() {
            return switch (PROFILE) {
                case MATCH -> MATCH_MAX_ANGULAR;
                case SIM   -> SIM_MAX_ANGULAR;
            };
        }

        
    }
    public static final class Vision{
        // Photon Vision
        public static final Transform3d CAMERA_TO_ROBOT =
            new Transform3d(
                new Translation3d(0.381, 0.0, 0.1905), // meters: forward, left, up
                new Rotation3d(0,0,0)     // radians: pitch, yaw, roll
            );

        // Standard deviations for measurement trust
        public static final Matrix<N3, N1> PHOTONVISION_STD_DEVS = VecBuilder.fill(
            0.02,  // 2 cm X
            0.02,  // 2 cm Y
            0.035  // ~2 degrees
        );

        // --- April Tags ---
        public static final AprilTagFieldLayout APRILTAG_LAYOUT =
            AprilTagFields.k2026RebuiltAndymark.loadAprilTagLayoutField();
    }
}
