//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS


package org.team1507.robot.subsystems;

import static org.wpilib.units.Units.*;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.MomentOfInertia;
import org.wpilib.units.measure.Voltage;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;

import org.team1507.lib.core.impl.ctre.CtreMotorConfigurator;
import org.team1507.lib.core.impl.ctre.Motor1507;
import org.team1507.lib.core.swerve.SwerveModule1507;
import org.team1507.lib.core.swerve.SwerveModule1507.MathConfig;
import org.team1507.lib.core.util.MotorConfig;
import org.team1507.lib.core.util.MotorConfig.ControlMode;
import org.team1507.robot.Constants;
import org.team1507.robot.Constants.kSwerve;

// ─────────────────────────────────────────────────────────────────────────────
// SwerveConfig
//
// Everything about THIS robot's drivetrain hardware, in one place:
//
//   1. Tuner X paste zone — CAN IDs, encoder offsets, gear ratios, wheel size,
//      gains, module positions. Paste Tuner X's generated values here.
//   2. 1507 additions — settings Tuner X doesn't generate (FOC, supply limits).
//   3. Motor configs and module creation — built from 1 and 2.
//   4. Build-time checks — SwerveConfigTest fails the build if a pasted value
//      looks wrong (units, duplicate CAN IDs, modules in the wrong corner).
//
// How the drivetrain DRIVES (odometry, driving, commands) lives in Swerve.java.
// Season setup: work through the Swerve section of the "Season Setup
// Checklist" wiki page; its items match the TODO(SEASON SWERVE-n) tags here.
// ─────────────────────────────────────────────────────────────────────────────
public final class SwerveConfig {

    private SwerveConfig() {}

    // ╔══════════════════════════════════════════════════════════════════╗
    // ║                 TUNER X PASTE ZONE — TunerConstants              ║
    // ║                                                                  ║
    // ║  The fields below use the SAME names, types, and units as the    ║
    // ║  TunerConstants.java file that Phoenix Tuner X's swerve project  ║
    // ║  generator creates. To update from Tuner X:                      ║
    // ║                                                                  ║
    // ║   1. Generate the project in Tuner X and open TunerConstants.java║
    // ║   2. Copy from "private static final Slot0Configs steerGains"    ║
    // ║      down to the last module's kBackRightYPos line.              ║
    // ║   3. Paste it over the matching lines below.                     ║
    // ║   4. Do NOT paste these generator lines; we don't use them:      ║
    // ║        kCANBus            (bus is Constants.CAN_BUS)             ║
    // ║        DrivetrainConstants, ConstantCreator, the FrontLeft/...   ║
    // ║        SwerveModuleConstants objects, createDrivetrain(), and    ║
    // ║        the TunerSwerveDrivetrain class                           ║
    // ║   5. Keep TUNED gains: if SysId has replaced the generator's     ║
    // ║      default steerGains/driveGains, don't paste over them.       ║
    // ║   6. ./gradlew build. The checks at the bottom of this file fail ║
    // ║      with a clear message if the paste picked a setting we       ║
    // ║      don't support (TorqueCurrentFOC output, TalonFXS motors).   ║
    // ║                                                                  ║
    // ║  Units match CTRE exactly:                                       ║
    // ║    driveGains: volts per DRIVE MOTOR rotation/sec                ║
    // ║    steerGains: volts per MODULE rotation (CANcoder)              ║
    // ║    kEncoderOffset: CANcoder MagnetOffset, written to the device  ║
    // ║                                                                  ║
    // ║  Settings the generator doesn't have (supply limits, FOC) live   ║
    // ║  in the "1507 ADDITIONS" block after this one. Pasting never     ║
    // ║  touches them.                                                   ║
    // ╚══════════════════════════════════════════════════════════════════╝
    //
    // SEASON CHECKLIST: every new robot must work through the Swerve section of
    // the "Season Setup Checklist" wiki page. Each item has a matching
    // TODO(SEASON SWERVE-n) tag in the code (search for "TODO(SEASON").
    // Pasting from Tuner X replaces the TODO lines inside the paste zone; that
    // is expected, because the paste is how those items get done.
    //
    //   SWERVE-1  Hardware: module type, drive/steer motors, Pro licenses
    //   SWERVE-2  CAN bus port the drivetrain is wired to    (Constants.CAN_BUS)
    //   SWERVE-3  Tuner X generator: IDs, offsets, inversions, module positions
    //   SWERVE-4  Drive gear ratio and coupling ratio
    //   SWERVE-5  Wheel radius and kSpeedAt12Volts
    //   SWERVE-6  Verify on blocks: steering, drive direction, gyro, odometry
    //   SWERVE-7  Tune gains with SysId                      (driveGains, steerGains)
    //   SWERVE-8  Current limits and slip current            (kSlipCurrent, *_SUPPLY_LIMIT)
    //   SWERVE-9  Driving/auto tuning knobs                  (Constants.kSwerve.kTuning)

    /** Pasted Tuner X constants. Private fields work because SwerveConfig is the outer class. */
    @SuppressWarnings("unused") // generator fields we don't read yet (sim inertia, pigeon configs)
    private static final class TunerConstants {

        // TODO(SEASON SWERVE-1): confirm the hardware. Current values are for the
        // 2027 robot: SDS MK5n modules, Kraken X60 drive, Kraken X44 steer, Phoenix Pro.
        //
        // TODO(SEASON SWERVE-7): starting gains are from Team 340's 2026 robot, which
        // runs the same MK5n + Kraken X60 (FOC) combination. Tune with SysId.

        // Both sets of gains need to be tuned to your individual robot.

        // The steer motor uses any SwerveModule.SteerRequestType control request with the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        private static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(100).withKI(0).withKD(0.2)
            .withKS(0).withKV(0).withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        private static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(0.25).withKI(0).withKD(0)
            .withKS(0).withKV(0.125);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        // The type of motor used for the drive motor
        private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
        // The type of motor used for the drive motor
        private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

        // The remote sensor feedback type to use for the steer motors;
        // When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
        // TODO(SEASON SWERVE-1): FusedCANcoder needs Phoenix Pro on the steer motors.
        private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        // TODO(SEASON SWERVE-8): 80 A matches 340's MK5n drive stator limit. Measure
        // our real slip current on carpet.
        private static final Current kSlipCurrent = Amps.of(80.0);

        // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
        // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
        private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
        private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Swerve azimuth does not require much torque output, so we can set a relatively low
                    // stator current limit to help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true)
            );
        private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
        // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
        private static final Pigeon2Configuration pigeonConfigs = null;

        // Theoretical free speed (m/s) at 12 V applied output;
        // This needs to be tuned to your individual robot
        // TODO(SEASON SWERVE-5): Kraken X60 FOC free speed 5800 RPM / 6.12 ratio on a
        // 4" wheel = 5.04 m/s. SwerveConfigTest fails if this doesn't match the
        // gear ratio and wheel radius.
        public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.04);

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        // TODO(SEASON SWERVE-4): 3.57 is the MK4i value. Take the MK5n value from
        // Tuner X's generator.
        private static final double kCoupleRatio = 3.5714285714285716;

        // TODO(SEASON SWERVE-4): the MK5n kit includes three drive ratios. 6.12 is our
        // 2026 ratio. Confirm which one is installed and update this to match.
        private static final double kDriveGearRatio = 6.122448979591837;
        // MK5n steering ratio: 287:11 (from SDS).
        private static final double kSteerGearRatio = 287.0 / 11.0;
        // TODO(SEASON SWERVE-5): MK5n with 4" x 2.25" molded spike grip wheels. 2.0"
        // is the NEW radius; tread wears down, so re-measure (or run wheel-radius
        // calibration) during the season and adjust DRIVE_METERS_SCALE in
        // Constants.kSwerve.kTuning.
        private static final Distance kWheelRadius = Inches.of(2.0);

        // TODO(SEASON SWERVE-6): verify drive direction on blocks (all wheels spin so
        // the robot moves forward when the stick is pushed forward).
        private static final boolean kInvertLeftSide = false;
        private static final boolean kInvertRightSide = true;

        private static final int kPigeonId = 30;

        // These are only used for simulation
        private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
        private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
        // Simulated voltage necessary to overcome friction
        private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
        private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

        // TODO(SEASON SWERVE-3): every value from here down (CAN IDs, encoder offsets,
        // inversions, module positions) is from the 2026 robot. Run Tuner X's swerve
        // generator on the new robot and paste its values here.

        // Front Left
        private static final int kFrontLeftDriveMotorId = 7;
        private static final int kFrontLeftSteerMotorId = 8;
        private static final int kFrontLeftEncoderId = 9;
        private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.129150390625);
        private static final boolean kFrontLeftSteerMotorInverted = false;
        private static final boolean kFrontLeftEncoderInverted = false;

        private static final Distance kFrontLeftXPos = Inches.of(10.7375);
        private static final Distance kFrontLeftYPos = Inches.of(10.7375);

        // Front Right
        private static final int kFrontRightDriveMotorId = 1;
        private static final int kFrontRightSteerMotorId = 2;
        private static final int kFrontRightEncoderId = 3;
        private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.28515625);
        private static final boolean kFrontRightSteerMotorInverted = false;
        private static final boolean kFrontRightEncoderInverted = false;

        private static final Distance kFrontRightXPos = Inches.of(10.7375);
        private static final Distance kFrontRightYPos = Inches.of(-10.7375);

        // Back Left
        private static final int kBackLeftDriveMotorId = 4;
        private static final int kBackLeftSteerMotorId = 5;
        private static final int kBackLeftEncoderId = 6;
        private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.436767578125);
        private static final boolean kBackLeftSteerMotorInverted = false;
        private static final boolean kBackLeftEncoderInverted = false;

        private static final Distance kBackLeftXPos = Inches.of(-10.7375);
        private static final Distance kBackLeftYPos = Inches.of(10.7375);

        // Back Right
        private static final int kBackRightDriveMotorId = 10;
        private static final int kBackRightSteerMotorId = 11;
        private static final int kBackRightEncoderId = 12;
        private static final Angle kBackRightEncoderOffset = Rotations.of(0.138671875);
        private static final boolean kBackRightSteerMotorInverted = false;
        private static final boolean kBackRightEncoderInverted = false;

        private static final Distance kBackRightXPos = Inches.of(-10.7375);
        private static final Distance kBackRightYPos = Inches.of(-10.7375);
    }

    // ╔══════════════════════════════════════════════════════════════════╗
    // ║                        1507 ADDITIONS                            ║
    // ║                                                                  ║
    // ║  Settings Tuner X's generator does not produce. Pasting a new    ║
    // ║  TunerConstants block never changes these.                       ║
    // ╚══════════════════════════════════════════════════════════════════╝

    /**
     * Use FOC commutation on all swerve motors. Requires Phoenix Pro on every
     * drive and steer motor (we are licensed). FOC gives about 15% more power
     * and more torque per amp. Tuner X's generated code gets this from CTRE's
     * swerve requests; our motors get it from here.
     */
    // TODO(SEASON SWERVE-1): set false if the swerve motors are not Pro-licensed.
    private static final boolean USE_FOC = true;

    /**
     * Supply (battery-side) current limits. These are the main brownout
     * protection: they cap how hard each motor can pull on the battery.
     * Starting values match Team 340's MK5n robot (4 drive x 28 A = 112 A
     * total for driving). Raise them only with match logs showing voltage
     * stays healthy.
     */
    // TODO(SEASON SWERVE-8): review against match logs (battery voltage, brownouts).
    private static final Current DRIVE_SUPPLY_LIMIT = Amps.of(28.0);
    private static final Current STEER_SUPPLY_LIMIT = Amps.of(40.0);

    /** Kraken X60 free speed with FOC (rotations/sec): 5800 RPM. Used to sanity-check kSpeedAt12Volts. */
    // TODO(SEASON SWERVE-1): update if the drive motor changes (non-FOC X60 = 6000 RPM).
    private static final double KRAKEN_X60_FOC_FREE_RPS = 5800.0 / 60.0;

    // ─────────────────────────────────────────────────────────────────
    // Values other code reads (computed from the paste zone)
    // ─────────────────────────────────────────────────────────────────

    /** Maximum translational speed (m/s): Tuner X's kSpeedAt12Volts. */
    public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    /** Distance from robot center to a module (m). Used by NodeBoundsTest for robot size. */
    public static final double DRIVE_BASE_RADIUS = Math.hypot(
        TunerConstants.kFrontLeftXPos.in(Meters), TunerConstants.kFrontLeftYPos.in(Meters));

    /** Maximum chassis angular rate (rad/s) = v_max / drive-base radius. */
    public static final double MAX_ANGULAR_RATE = MAX_SPEED / DRIVE_BASE_RADIUS;

    // ============================================================
    // Motor configs — built ONCE from the paste zone, then copied per module
    // ============================================================

    /**
     * Drive motors: velocity control in motor rps, gains straight from
     * driveGains. Each module copies this and sets its own inversion.
     */
    private static final MotorConfig DRIVE_CONFIG = buildDriveConfig();

    /**
     * Steer motors: position control on the CANcoder in module rotations.
     * RotorToSensorRatio = steer ratio, so FusedCANcoder can blend the CANcoder
     * with the motor's own fast encoder (Pro). Each module copies this and
     * sets its own CANcoder ID and inversion.
     */
    private static final MotorConfig STEER_CONFIG = buildSteerConfig();

    private static MotorConfig buildDriveConfig() {
        Slot0Configs g = TunerConstants.driveGains;
        MotorConfig.Builder b = MotorConfig.builder(ControlMode.VELOCITY)
            .withPID(g.kP, g.kI, g.kD)
            .withFeedforward(g.kS, g.kV, g.kA)
            .withStatorCurrentLimit(TunerConstants.kSlipCurrent)
            .withSupplyCurrentLimit(DRIVE_SUPPLY_LIMIT)
            .withBrake();
        if (USE_FOC) b.withFOC();
        return b.build();
    }

    private static MotorConfig buildSteerConfig() {
        Slot0Configs g = TunerConstants.steerGains;
        FeedbackSensorSourceValue source = switch (TunerConstants.kSteerFeedbackType) {
            case FusedCANcoder  -> FeedbackSensorSourceValue.FusedCANcoder;
            case SyncCANcoder   -> FeedbackSensorSourceValue.SyncCANcoder;
            case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
            default -> throw new IllegalStateException(
                "Swerve: unsupported kSteerFeedbackType " + TunerConstants.kSteerFeedbackType);
        };
        double statorLimit = TunerConstants.steerInitialConfigs.CurrentLimits.StatorCurrentLimit;
        MotorConfig.Builder b = MotorConfig.builder(ControlMode.POSITION)
            .withPID(g.kP, g.kI, g.kD)
            .withFeedforward(g.kS, g.kV, g.kA)
            .withStaticFeedforwardSign(g.StaticFeedforwardSign)
            .withStatorCurrentLimit(statorLimit)
            .withSupplyCurrentLimit(STEER_SUPPLY_LIMIT)
            .withFeedbackSensor(source)
            .withRotorToSensorRatio(TunerConstants.kSteerGearRatio)
            .withSensorToMechanismRatio(1.0)
            .withContinuousWrap()
            // Stall = high current while not moving. The default 60 A threshold
            // equals the steer stator limit, so current could never exceed it.
            .withStallCurrentThreshold(0.75 * statorLimit)
            .withBrake();
        if (USE_FOC) b.withFOC();
        return b.build();
    }

    /** Wheel/steer math shared by all four modules. */
    private static final MathConfig MODULE_MATH = new MathConfig(
        TunerConstants.kDriveGearRatio,
        TunerConstants.kCoupleRatio,
        TunerConstants.kWheelRadius.in(Meters)
    );

    // ============================================================
    // What Swerve.java uses
    // ============================================================

    /** The front-left module, built from the paste zone. */
    static SwerveModule1507 frontLeft() {
        return createModule("FrontLeft",
            TunerConstants.kFrontLeftDriveMotorId, TunerConstants.kFrontLeftSteerMotorId,
            TunerConstants.kFrontLeftEncoderId, TunerConstants.kFrontLeftEncoderOffset,
            TunerConstants.kInvertLeftSide, TunerConstants.kFrontLeftSteerMotorInverted,
            TunerConstants.kFrontLeftEncoderInverted);
    }

    /** The front-right module, built from the paste zone. */
    static SwerveModule1507 frontRight() {
        return createModule("FrontRight",
            TunerConstants.kFrontRightDriveMotorId, TunerConstants.kFrontRightSteerMotorId,
            TunerConstants.kFrontRightEncoderId, TunerConstants.kFrontRightEncoderOffset,
            TunerConstants.kInvertRightSide, TunerConstants.kFrontRightSteerMotorInverted,
            TunerConstants.kFrontRightEncoderInverted);
    }

    /** The back-left module, built from the paste zone. */
    static SwerveModule1507 backLeft() {
        return createModule("BackLeft",
            TunerConstants.kBackLeftDriveMotorId, TunerConstants.kBackLeftSteerMotorId,
            TunerConstants.kBackLeftEncoderId, TunerConstants.kBackLeftEncoderOffset,
            TunerConstants.kInvertLeftSide, TunerConstants.kBackLeftSteerMotorInverted,
            TunerConstants.kBackLeftEncoderInverted);
    }

    /** The back-right module, built from the paste zone. */
    static SwerveModule1507 backRight() {
        return createModule("BackRight",
            TunerConstants.kBackRightDriveMotorId, TunerConstants.kBackRightSteerMotorId,
            TunerConstants.kBackRightEncoderId, TunerConstants.kBackRightEncoderOffset,
            TunerConstants.kInvertRightSide, TunerConstants.kBackRightSteerMotorInverted,
            TunerConstants.kBackRightEncoderInverted);
    }

    /** Module positions in FL, FR, BL, BR order (the order Swerve passes modules in). */
    static SwerveDriveKinematics kinematics() {
        return new SwerveDriveKinematics(
            new Translation2d(TunerConstants.kFrontLeftXPos,  TunerConstants.kFrontLeftYPos),
            new Translation2d(TunerConstants.kFrontRightXPos, TunerConstants.kFrontRightYPos),
            new Translation2d(TunerConstants.kBackLeftXPos,   TunerConstants.kBackLeftYPos),
            new Translation2d(TunerConstants.kBackRightXPos,  TunerConstants.kBackRightYPos)
        );
    }

    /** CAN ID of the Pigeon 2 gyro. */
    static int pigeonId() {
        return TunerConstants.kPigeonId;
    }

    // ============================================================
    // Hardware setup
    // ============================================================

    /**
     * Fails fast if the pasted TunerConstants picked a setting our swerve code
     * does not support, instead of silently driving with the wrong units.
     */
    static void checkPastedSettings() {
        if (TunerConstants.kDriveClosedLoopOutput != ClosedLoopOutputType.Voltage
                || TunerConstants.kSteerClosedLoopOutput != ClosedLoopOutputType.Voltage) {
            throw new IllegalStateException(
                "Swerve: 1507 swerve supports Voltage closed-loop output only. "
                + "Regenerate in Tuner X with Voltage, or add TorqueCurrentFOC support "
                + "(gains would be in amps, not volts).");
        }
        if (TunerConstants.kDriveMotorType != DriveMotorArrangement.TalonFX_Integrated
                || TunerConstants.kSteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
            throw new IllegalStateException(
                "Swerve: 1507 swerve supports TalonFX-integrated motors (Kraken X60/X44) only.");
        }
    }

    /**
     * Sanity-checks the pasted TunerConstants and 1507 additions.
     *
     * <p>Returns one plain-English message per problem, or an empty list. The
     * unit test {@code SwerveConfigTest} fails the build if this is not empty, and
     * the robot also prints each problem to the Driver Station at startup.
     *
     * <p>Each check exists because the mistake is easy to make and the compiler
     * can't catch it: wrong units, a typo'd CAN ID, a module in the wrong slot.
     */
    static List<String> configProblems() {
        List<String> problems = new ArrayList<>();

        // --- Units: drive kV must be volts per MOTOR rps. ~12 V / free speed = 0.12.
        // The 2026 code had kV = 2.75 (volts per m/s), which saturated the drive motors.
        double kV = TunerConstants.driveGains.kV;
        if (kV < 0.05 || kV > 0.25) {
            problems.add("driveGains.kV = " + kV + " is outside 0.05-0.25 V per motor rps. "
                + "Expected about 12 V / motor free speed (~0.12). Is it in m/s units?");
        }
        if (TunerConstants.driveGains.kP > 1.0) {
            problems.add("driveGains.kP = " + TunerConstants.driveGains.kP
                + " is very high for volts per motor rps of error (340 uses 0.25).");
        }
        if (TunerConstants.steerGains.kP <= 0.0) {
            problems.add("steerGains.kP must be positive or the wheels will not steer.");
        }

        // --- Mechanics
        checkGearRatio(problems, "kDriveGearRatio", TunerConstants.kDriveGearRatio);
        checkGearRatio(problems, "kSteerGearRatio", TunerConstants.kSteerGearRatio);
        double wheelIn = TunerConstants.kWheelRadius.in(Inches);
        if (wheelIn < 1.5 || wheelIn > 2.5) {
            problems.add("kWheelRadius = " + wheelIn + " in. Radius, not diameter? (a 4 in wheel has a 2 in radius)");
        }
        double expectedSpeed = KRAKEN_X60_FOC_FREE_RPS / TunerConstants.kDriveGearRatio
            * 2.0 * Math.PI * TunerConstants.kWheelRadius.in(Meters);
        if (Math.abs(MAX_SPEED - expectedSpeed) > 0.10 * expectedSpeed) {
            problems.add(String.format(
                "kSpeedAt12Volts = %.2f m/s, but gear ratio and wheel size give %.2f m/s. "
                + "Did the ratio or wheel change without updating it?", MAX_SPEED, expectedSpeed));
        }

        // --- CAN IDs: no duplicates across the drivetrain (all share one CAN bus).
        int[] ids = {
            TunerConstants.kFrontLeftDriveMotorId,  TunerConstants.kFrontLeftSteerMotorId,  TunerConstants.kFrontLeftEncoderId,
            TunerConstants.kFrontRightDriveMotorId, TunerConstants.kFrontRightSteerMotorId, TunerConstants.kFrontRightEncoderId,
            TunerConstants.kBackLeftDriveMotorId,   TunerConstants.kBackLeftSteerMotorId,   TunerConstants.kBackLeftEncoderId,
            TunerConstants.kBackRightDriveMotorId,  TunerConstants.kBackRightSteerMotorId,  TunerConstants.kBackRightEncoderId,
            TunerConstants.kPigeonId
        };
        Set<Integer> seen = new HashSet<>();
        for (int id : ids) {
            if (!seen.add(id)) {
                problems.add("CAN ID " + id + " is used by more than one drivetrain device.");
            }
        }

        // --- Encoder offsets are CANcoder MagnetOffset values, which must be within one rotation.
        Angle[] offsets = {
            TunerConstants.kFrontLeftEncoderOffset, TunerConstants.kFrontRightEncoderOffset,
            TunerConstants.kBackLeftEncoderOffset,  TunerConstants.kBackRightEncoderOffset
        };
        for (Angle offset : offsets) {
            double rot = offset.in(Rotations);
            if (rot < -1.0 || rot > 1.0) {
                problems.add("Encoder offset " + rot + " rotations is outside -1..1. Degrees instead of rotations?");
            }
        }

        // --- Module positions: kinematics assumes FL, FR, BL, BR order (+X forward, +Y left).
        checkQuadrant(problems, "FrontLeft",  TunerConstants.kFrontLeftXPos,  TunerConstants.kFrontLeftYPos,   1,  1);
        checkQuadrant(problems, "FrontRight", TunerConstants.kFrontRightXPos, TunerConstants.kFrontRightYPos,  1, -1);
        checkQuadrant(problems, "BackLeft",   TunerConstants.kBackLeftXPos,   TunerConstants.kBackLeftYPos,   -1,  1);
        checkQuadrant(problems, "BackRight",  TunerConstants.kBackRightXPos,  TunerConstants.kBackRightYPos,  -1, -1);

        // --- The MotorConfigs built from the paste zone pass the general motor checks.
        for (String problem : DRIVE_CONFIG.problems()) {
            problems.add("drive motor config: " + problem);
        }
        for (String problem : STEER_CONFIG.problems()) {
            problems.add("steer motor config: " + problem);
        }

        // --- Current limits: supply limits are our brownout protection.
        if (DRIVE_SUPPLY_LIMIT.in(Amps) <= 0.0 || STEER_SUPPLY_LIMIT.in(Amps) <= 0.0) {
            problems.add("Supply current limits must be positive.");
        }
        if (DRIVE_SUPPLY_LIMIT.in(Amps) > TunerConstants.kSlipCurrent.in(Amps)) {
            problems.add("DRIVE_SUPPLY_LIMIT is above kSlipCurrent (the drive stator limit).");
        }

        return problems;
    }

    /**
     * Gear ratios are motor turns per wheel/module turn, so they are always
     * greater than 1. A value below 1 was almost certainly typed upside down.
     *
     * <p>(This is a method on purpose: with the paste zone's fixed values, an
     * inline {@code if} would be flagged by the IDE as "dead code", because the
     * compiler can see it's false today. It still matters after the next paste.)
     */
    private static void checkGearRatio(List<String> problems, String name, double ratio) {
        if (ratio <= 1.0) {
            problems.add(name + " = " + ratio + ". Gear ratios are motor turns per wheel/module "
                + "turn (> 1). Typed upside down?");
        }
    }

    private static void checkQuadrant(
        List<String> problems, String name, Distance x, Distance y, int xSign, int ySign
    ) {
        if (Math.signum(x.in(Meters)) != xSign || Math.signum(y.in(Meters)) != ySign) {
            problems.add(name + " position (" + x.in(Inches) + " in, " + y.in(Inches)
                + " in) is in the wrong corner. +X is forward, +Y is left.");
        }
    }

    /**
     * Builds one swerve module from pasted TunerConstants values, the same way
     * CTRE's generated drivetrain would: the encoder offset is written to the
     * CANcoder itself, and the steer motor closes its loop on that CANcoder.
     */
    private static SwerveModule1507 createModule(
        String name,
        int driveId, int steerId, int encoderId,
        Angle encoderOffset,
        boolean driveInverted, boolean steerInverted, boolean encoderInverted
    ) {
        // CANcoder: store the offset ON the device, so its absolute position
        // already reads 0 when the wheel points forward. The steer motor's
        // closed loop and our odometry both read this one corrected value.
        CANcoder encoder = new CANcoder(encoderId, Constants.CAN_BUS);
        CANcoderConfiguration encoderConfig = TunerConstants.encoderInitialConfigs;
        encoderConfig.MagnetSensor.MagnetOffset = encoderOffset.in(Rotations);
        encoderConfig.MagnetSensor.SensorDirection = encoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
        CtreMotorConfigurator.applyWithRetry("CANcoder " + encoderId,
            () -> encoder.getConfigurator().apply(encoderConfig));

        // Same drive/steer configs for every module; only inversion and the
        // steer motor's CANcoder ID differ.
        MotorConfig driveConfig = MotorConfig.copy(DRIVE_CONFIG)
            .inverted(driveInverted)
            .build();
        MotorConfig steerConfig = MotorConfig.copy(STEER_CONFIG)
            .withRemoteSensorId(encoderId)
            .inverted(steerInverted)
            .build();

        Motor1507 drive = new Motor1507("Swerve/" + name + "/Drive", Motor1507.Type.FX,
            driveId, Constants.CAN_BUS, driveConfig);
        Motor1507 steer = new Motor1507("Swerve/" + name + "/Steer", Motor1507.Type.FX,
            steerId, Constants.CAN_BUS, steerConfig);

        return new SwerveModule1507(name, drive, steer, encoder, MODULE_MATH, kSwerve.kTuning.DRIVE_METERS_SCALE);
    }
}
