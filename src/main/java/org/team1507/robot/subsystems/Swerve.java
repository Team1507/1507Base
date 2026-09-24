//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.subsystems;

import java.util.function.Supplier;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.*;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.MomentOfInertia;
import org.wpilib.units.measure.Voltage;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.framework.RobotBase;
import org.wpilib.system.Timer;
import org.wpilib.command2.Command;

import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;

import org.team1507.lib.core.framework.Subsystem1507;
import org.team1507.lib.core.impl.ctre.CtreMotorConfigurator;
import org.team1507.lib.core.impl.ctre.Motor1507;
import org.team1507.lib.core.logging.Telemetry;
import org.team1507.lib.core.swerve.SwerveModule1507;
import org.team1507.lib.core.swerve.SwerveModule1507.MathConfig;
import org.team1507.lib.core.util.MotorConfig;
import org.team1507.lib.core.util.MotorConfig.ControlMode;
import org.team1507.robot.Constants;
import org.team1507.robot.Constants.kSwerve;
import static org.team1507.robot.Constants.kSwerve.kTuning.*;

public final class Swerve extends Subsystem1507 {

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
    // ║   6. ./gradlew build. The checks in the Swerve constructor fail  ║
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

    /** Pasted Tuner X constants. Private fields work because Swerve is the outer class. */
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

    // ------------------------------------------------------------
    // Modules
    // ------------------------------------------------------------

    private final SwerveModule1507 frontLeft;
    private final SwerveModule1507 frontRight;
    private final SwerveModule1507 backLeft;
    private final SwerveModule1507 backRight;

    // ------------------------------------------------------------
    // Kinematics & odometry
    // ------------------------------------------------------------

    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;

    // ------------------------------------------------------------
    // Sensors
    // ------------------------------------------------------------

    private final Pigeon2 pigeon;
    private final StatusSignal<Angle> yaw;
    private final StatusSignal<AngularVelocity> yawRate;

    // ------------------------------------------------------------
    // Cached state
    // ------------------------------------------------------------

    private Pose2d pose = new Pose2d();

    private final SwerveModuleVelocity[] moduleStates = new SwerveModuleVelocity[4];
    private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    private final double maxSpeedMetersPerSecond;
    private final double maxAngularMetersPerSecond;


    // ------------------------------------------------------------
    // Simulated Data
    // ------------------------------------------------------------

    private double simHeadingRadians = 0.0;
    private ChassisVelocities lastCommandedSpeeds = new ChassisVelocities();

    // ------------------------------------------------------------
    // CAN Signal Batch
    // ------------------------------------------------------------

    private final BaseStatusSignal[] allSignals;

    // ============================================================
    // Constructor
    // ============================================================

    public Swerve() {
        super("Swerve");
        checkPastedSettings();
        for (String problem : configProblems()) {
            DriverStationErrors.reportWarning("[Swerve config] " + problem, false);
        }

        MathConfig math = new MathConfig(
            TunerConstants.kDriveGearRatio, TunerConstants.kSteerGearRatio,
            TunerConstants.kCoupleRatio, TunerConstants.kWheelRadius.in(Meters)
        );

        this.frontLeft = createModule("FrontLeft",
            TunerConstants.kFrontLeftDriveMotorId, TunerConstants.kFrontLeftSteerMotorId,
            TunerConstants.kFrontLeftEncoderId, TunerConstants.kFrontLeftEncoderOffset,
            TunerConstants.kInvertLeftSide, TunerConstants.kFrontLeftSteerMotorInverted,
            TunerConstants.kFrontLeftEncoderInverted, math);

        this.frontRight = createModule("FrontRight",
            TunerConstants.kFrontRightDriveMotorId, TunerConstants.kFrontRightSteerMotorId,
            TunerConstants.kFrontRightEncoderId, TunerConstants.kFrontRightEncoderOffset,
            TunerConstants.kInvertRightSide, TunerConstants.kFrontRightSteerMotorInverted,
            TunerConstants.kFrontRightEncoderInverted, math);

        this.backLeft = createModule("BackLeft",
            TunerConstants.kBackLeftDriveMotorId, TunerConstants.kBackLeftSteerMotorId,
            TunerConstants.kBackLeftEncoderId, TunerConstants.kBackLeftEncoderOffset,
            TunerConstants.kInvertLeftSide, TunerConstants.kBackLeftSteerMotorInverted,
            TunerConstants.kBackLeftEncoderInverted, math);

        this.backRight = createModule("BackRight",
            TunerConstants.kBackRightDriveMotorId, TunerConstants.kBackRightSteerMotorId,
            TunerConstants.kBackRightEncoderId, TunerConstants.kBackRightEncoderOffset,
            TunerConstants.kInvertRightSide, TunerConstants.kBackRightSteerMotorInverted,
            TunerConstants.kBackRightEncoderInverted, math);

        this.pigeon = new Pigeon2(TunerConstants.kPigeonId, Constants.CAN_BUS);
        this.maxSpeedMetersPerSecond = MAX_SPEED;
        this.maxAngularMetersPerSecond = MAX_ANGULAR_RATE;

        this.yaw = pigeon.getYaw();
        this.yawRate = pigeon.getAngularVelocityZWorld();

        // The heading feeds odometry and field-relative driving, so it must update
        // every loop. Without this, optimizeBusUtilizationForAll() below would drop
        // yaw to 4 Hz (it slows every signal that has no explicit frequency).
        BaseStatusSignal.setUpdateFrequencyForAll(100.0, yaw, yawRate);

        this.kinematics = new SwerveDriveKinematics(
            new Translation2d(TunerConstants.kFrontLeftXPos,  TunerConstants.kFrontLeftYPos),
            new Translation2d(TunerConstants.kFrontRightXPos, TunerConstants.kFrontRightYPos),
            new Translation2d(TunerConstants.kBackLeftXPos,   TunerConstants.kBackLeftYPos),
            new Translation2d(TunerConstants.kBackRightXPos,  TunerConstants.kBackRightYPos)
        );

        this.poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            getHeading(),
            getModulePositions(),
            pose,
            kSwerve.kTuning.ODOMETRY_STD_DEV,
            kSwerve.kTuning.VISION_STD_DEV
        );

        // Disable unused CAN signals on every swerve device to avoid flooding the bus.
        // Only signals given an explicit frequency above keep their rate.
        ParentDevice.optimizeBusUtilizationForAll(
            4.0,
            frontLeft.getDriveDevice(),  frontLeft.getSteerDevice(),  frontLeft.getEncoderDevice(),
            frontRight.getDriveDevice(), frontRight.getSteerDevice(), frontRight.getEncoderDevice(),
            backLeft.getDriveDevice(),   backLeft.getSteerDevice(),   backLeft.getEncoderDevice(),
            backRight.getDriveDevice(),  backRight.getSteerDevice(),  backRight.getEncoderDevice(),
            pigeon
        );

        // Build the master signal array once: every module's signals + Pigeon2 yaw and yaw rate.
        java.util.List<BaseStatusSignal> signals = new java.util.ArrayList<>();
        for (SwerveModule1507 m : new SwerveModule1507[] { frontLeft, frontRight, backLeft, backRight }) {
            signals.addAll(java.util.List.of(m.getAllSignals()));
        }
        signals.add(yaw);
        signals.add(yawRate);
        allSignals = signals.toArray(BaseStatusSignal[]::new);

        Telemetry.set("Swerve/Initialized", true);
    }

    // ============================================================
    // Hardware setup
    // ============================================================

    /**
     * Fails fast if the pasted TunerConstants picked a setting our swerve code
     * does not support, instead of silently driving with the wrong units.
     */
    private static void checkPastedSettings() {
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
    static java.util.List<String> configProblems() {
        java.util.List<String> problems = new java.util.ArrayList<>();

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
        if (TunerConstants.kDriveGearRatio <= 1.0 || TunerConstants.kSteerGearRatio <= 1.0) {
            problems.add("Gear ratios must be motor rotations per wheel/module rotation (> 1).");
        }
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
        java.util.Set<Integer> seen = new java.util.HashSet<>();
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

        // --- Current limits: supply limits are our brownout protection.
        if (DRIVE_SUPPLY_LIMIT.in(Amps) <= 0.0 || STEER_SUPPLY_LIMIT.in(Amps) <= 0.0) {
            problems.add("Supply current limits must be positive.");
        }
        if (DRIVE_SUPPLY_LIMIT.in(Amps) > TunerConstants.kSlipCurrent.in(Amps)) {
            problems.add("DRIVE_SUPPLY_LIMIT is above kSlipCurrent (the drive stator limit).");
        }

        return problems;
    }

    private static void checkQuadrant(
        java.util.List<String> problems, String name, Distance x, Distance y, int xSign, int ySign
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
        boolean driveInverted, boolean steerInverted, boolean encoderInverted,
        MathConfig math
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

        Motor1507 drive = new Motor1507("Swerve/" + name + "/Drive", Motor1507.Type.FX,
            driveId, Constants.CAN_BUS, driveConfig(driveInverted));
        Motor1507 steer = new Motor1507("Swerve/" + name + "/Steer", Motor1507.Type.FX,
            steerId, Constants.CAN_BUS, steerConfig(encoderId, steerInverted));

        return new SwerveModule1507(name, drive, steer, encoder, math, kSwerve.kTuning.DRIVE_METERS_SCALE);
    }

    /** Drive motor: velocity control in motor rps, gains straight from driveGains. */
    private static MotorConfig driveConfig(boolean inverted) {
        Slot0Configs g = TunerConstants.driveGains;
        MotorConfig.Builder b = MotorConfig.builder(ControlMode.VELOCITY)
            .inverted(inverted)
            .withPID(g.kP, g.kI, g.kD)
            .withFeedforward(g.kS, g.kV, g.kA)
            .withStatorCurrentLimit(TunerConstants.kSlipCurrent)
            .withSupplyCurrentLimit(DRIVE_SUPPLY_LIMIT)
            .withBrake();
        if (USE_FOC) b.withFOC();
        return b.build();
    }

    /**
     * Steer motor: position control on the CANcoder in module rotations.
     * RotorToSensorRatio = steer ratio, so FusedCANcoder can blend the CANcoder
     * with the motor's own fast encoder (Pro).
     */
    private static MotorConfig steerConfig(int encoderId, boolean inverted) {
        Slot0Configs g = TunerConstants.steerGains;
        FeedbackSensorSourceValue source = switch (TunerConstants.kSteerFeedbackType) {
            case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
            case SyncCANcoder  -> FeedbackSensorSourceValue.SyncCANcoder;
            case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
            default -> throw new IllegalStateException(
                "Swerve: unsupported kSteerFeedbackType " + TunerConstants.kSteerFeedbackType);
        };
        MotorConfig.Builder b = MotorConfig.builder(ControlMode.POSITION)
            .inverted(inverted)
            .withPID(g.kP, g.kI, g.kD)
            .withFeedforward(g.kS, g.kV, g.kA)
            .withStatorCurrentLimit(
                Amps.of(TunerConstants.steerInitialConfigs.CurrentLimits.StatorCurrentLimit))
            .withSupplyCurrentLimit(STEER_SUPPLY_LIMIT)
            .withFeedbackSensor(source)
            .withRemoteSensorId(encoderId)
            .withRotorToSensorRatio(TunerConstants.kSteerGearRatio)
            .withSensorToMechanismRatio(1.0)
            .withContinuousWrap()
            // Stall = high current while not moving. The default 60 A threshold
            // equals the steer stator limit, so current could never exceed it.
            .withStallCurrentThreshold(
                0.75 * TunerConstants.steerInitialConfigs.CurrentLimits.StatorCurrentLimit)
            .withBrake();
        if (USE_FOC) b.withFOC();
        return b.build();
    }

    // ============================================================
    // Periodic
    // ============================================================

    @Override
    public void periodic() {
        if (RobotBase.isSimulation()) {
            // Integrate heading before poseEstimator.update() so the heading is fresh.
            // getHeading() reads simHeadingRadians directly in sim — no Pigeon signal needed.
            simHeadingRadians += lastCommandedSpeeds.omega * 0.02;
        } else {
            BaseStatusSignal.refreshAll(allSignals);
        }

        pose = poseEstimator.update(
            getHeading(),
            getModulePositions()
        );

        log("Pose", pose);
        log("DriveStalled", isAnyDriveStalled());
        log("SteerStalled", isAnySteerStalled());
        Telemetry.set("Swerve/ModuleStates", getModuleStates());
    }

    @Override
    public void simulationPeriodic() {
        frontLeft.simulationUpdate(0.02);
        frontRight.simulationUpdate(0.02);
        backLeft.simulationUpdate(0.02);
        backRight.simulationUpdate(0.02);
    }

    // ============================================================
    // Low-Level Control
    //
    // These are the raw driving methods called by commands.
    // Students generally don't call these directly — use a command.
    // ============================================================

    /**
     * Drives using field-relative ChassisVelocities (x = forward, y = left).
     * The kinematics layer converts these to individual module states.
     *
     * <p>{@code ChassisVelocities.discretize()} is applied before kinematics to correct
     * for the skew that occurs when the robot translates and rotates simultaneously
     * within a single 20 ms loop iteration. Without it, the robot arcs instead of
     * driving in a straight line.
     */
    public void drive(ChassisVelocities speeds) {
        lastCommandedSpeeds = speeds;
        SwerveModuleVelocity[] states = kinematics.toSwerveModuleVelocities(
            speeds.discretize(0.02)
        );
        // 2027: desaturateWheelVelocities returns a NEW array (it no longer edits
        // the one passed in), so the result must be assigned back.
        states = SwerveDriveKinematics.desaturateWheelVelocities(states, maxSpeedMetersPerSecond);

        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    /**
     * Drives using robot-relative ChassisVelocities.
     * Used by movement commands that already handle the field→robot conversion.
     *
     * <p>Applies {@code ChassisVelocities.discretize()} for the same skew correction
     * as {@link #drive(ChassisVelocities)}.
     */
    public void driveRobotRelative(ChassisVelocities speeds) {
        lastCommandedSpeeds = speeds;
        SwerveModuleVelocity[] states = kinematics.toSwerveModuleVelocities(
            speeds.discretize(0.02)
        );
        // 2027: desaturateWheelVelocities returns a NEW array (it no longer edits
        // the one passed in), so the result must be assigned back.
        states = SwerveDriveKinematics.desaturateWheelVelocities(states, maxSpeedMetersPerSecond);

        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    /** Stops all modules immediately. */
    public void stop() {
        lastCommandedSpeeds = new ChassisVelocities();
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /**
     * Sets all modules to the X configuration (FL/BR at +45°, FR/BL at -45°).
     *
     * <p>Uses {@code brakeToAngle()} on each module rather than
     * {@code setDesiredState()} so the steer angle is applied unconditionally.
     * {@code setDesiredState()} has an anti-jitter guard that skips steer
     * updates when speed is near zero — which would prevent the X pattern
     * from ever forming since brake() always passes speed = 0.
     *
     * <p>Called every loop by {@link #brakeCommand()} to hold the configuration.
     */
    public void brake() {
        frontLeft.brakeToAngle(Rotation2d.fromDegrees( 45));
        frontRight.brakeToAngle(Rotation2d.fromDegrees(-45));
        backLeft.brakeToAngle(Rotation2d.fromDegrees(-45));
        backRight.brakeToAngle(Rotation2d.fromDegrees( 45));
    }

    // ============================================================
    // Observation
    // ============================================================

    /** Returns the robot's current estimated field pose. */
    public Pose2d getPose() {
        return pose;
    }

    /** Returns the robot's current heading from the Pigeon2 (or simulated equivalent). */
    public Rotation2d getHeading() {
        if (RobotBase.isSimulation()) {
            return Rotation2d.fromRadians(simHeadingRadians);
        }
        // Latency-compensated: projects yaw forward by yaw rate x signal age.
        return Rotation2d.fromDegrees(BaseStatusSignal.getLatencyCompensatedValueAsDouble(yaw, yawRate));
    }

    /** Returns the robot's current robot-relative chassis speeds derived from module states. */
    public ChassisVelocities getChassisVelocities() {
        return kinematics.toChassisVelocities(getModuleStates());
    }

    /**
     * Returns the robot's velocity in field-relative coordinates.
     * Used by maintainHeadingToTarget for motion compensation.
     */
    public ChassisVelocities getFieldRelativeSpeeds() {
        return kinematics.toChassisVelocities(getModuleStates())
            .toFieldRelative(getHeading());
    }

    /** Returns the configured maximum translational speed in m/s. */
    public double getMaxSpeed() {
        return maxSpeedMetersPerSecond;
    }

    /** Returns the configured maximum angular rate in rad/s. */
    public double getMaxAngular() {
        return maxAngularMetersPerSecond;
    }

    // ============================================================
    // Vision & Localization
    // ============================================================

    /**
     * Feeds a vision-estimated pose into the pose estimator.
     *
     * @param visionPose        estimated robot pose from vision
     * @param timestampSeconds  FPGA timestamp of the measurement
     * @param stdDevs           measurement confidence [x (m), y (m), heading (rad)]
     */
    public void addVisionMeasurement(
        Pose2d visionPose,
        double timestampSeconds,
        Matrix<N3, N1> stdDevs
    ) {
        poseEstimator.addVisionMeasurement(
            visionPose,
            timestampSeconds,
            stdDevs
        );
    }

    /**
     * Resets the robot's heading to 0° so its current facing direction becomes
     * field-forward. Also resets the pose estimator's heading component.
     */
    public void zeroHeading() {
        pigeon.setYaw(0.0);
        // In simulation, getHeading() reads simHeadingRadians directly — the Phoenix
        // sim backend doesn't update it synchronously from setYaw(). Mirror the reset
        // here so simulation and real hardware behave identically.
        simHeadingRadians = 0.0;
        Rotation2d zero = new Rotation2d();
        pose = new Pose2d(pose.getTranslation(), zero);
        poseEstimator.resetPosition(zero, getModulePositions(), pose);
    }

    /**
     * Resets the robot's pose estimate to a known field position.
     * Used by autonomous routines and by QuestNavSubsystem after a confirmed pose reset.
     *
     * @param pose the new known pose
     */
    public void resetPose(Pose2d pose) {
        this.pose = pose;
        poseEstimator.resetPosition(
            getHeading(),
            getModulePositions(),
            pose
        );
    }

    // ============================================================
    // Fault interpretation
    // ============================================================

    /** Returns true if any drive motor is currently reporting a stall condition. */
    public boolean isAnyDriveStalled() {
        if (org.wpilib.framework.RobotBase.isSimulation()) return false;

        return frontLeft.isDriveStalled()
            || frontRight.isDriveStalled()
            || backLeft.isDriveStalled()
            || backRight.isDriveStalled();
    }

    /** Returns true if any steer motor is currently reporting a stall condition. */
    public boolean isAnySteerStalled() {
        if (org.wpilib.framework.RobotBase.isSimulation()) return false;

        return frontLeft.isSteerStalled()
            || frontRight.isSteerStalled()
            || backLeft.isSteerStalled()
            || backRight.isSteerStalled();
    }

    // ============================================================
    // Internal Helpers
    //
    // Private math utilities shared across multiple commands.
    // ============================================================

    /**
     * Simple proportional heading controller.
     * Returns an angular velocity (rad/s) to steer from current → desired rotation.
     *
     * <p>The error is wrapped to [-π, π] so the robot always takes the shortest path.
     * Output is clamped to the robot's maximum angular rate so large errors (e.g. 180°)
     * cannot produce physically impossible rotation commands.
     *
     * <p>HEADING_KP (5.5) means 1 radian of error produces 5.5 rad/s of correction.
     * Without clamping, a 180° error would produce ~17.3 rad/s — above the ~13.1 rad/s
     * physical maximum.
     */
    private double computeOmega(Rotation2d current, Rotation2d desired) {
        double error = MathUtil.angleModulus(desired.minus(current).getRadians());
        return Math.clamp(error * HEADING_KP, -maxAngularMetersPerSecond, maxAngularMetersPerSecond);
    }

    /**
     * Computes the direction the robot must face to look directly at a target.
     * Returns a Rotation2d pointing from robot's current XY → target's XY.
     * The target's own rotation field is ignored.
     */
    private Rotation2d computeHeadingToTarget(Pose2d robot, Pose2d target) {
        double dx = target.getX() - robot.getX();
        double dy = target.getY() - robot.getY();
        return new Rotation2d(Math.atan2(dy, dx));
    }

    private SwerveModuleVelocity[] getModuleStates() {
        moduleStates[0] = frontLeft.getState();
        moduleStates[1] = frontRight.getState();
        moduleStates[2] = backLeft.getState();
        moduleStates[3] = backRight.getState();
        return moduleStates;
    }

    private SwerveModulePosition[] getModulePositions() {
        modulePositions[0] = frontLeft.getPosition();
        modulePositions[1] = frontRight.getPosition();
        modulePositions[2] = backLeft.getPosition();
        modulePositions[3] = backRight.getPosition();
        return modulePositions;
    }

    // ╔══════════════════════════════════════════════════════════════════╗
    // ║                        COMMANDS                                  ║
    // ║                                                                  ║
    // ║  All swerve commands live here, directly on the subsystem.       ║
    // ║  They use run() / runOnce() / andThen() from SubsystemBase,      ║
    // ║  which automatically registers this subsystem as a requirement.  ║
    // ║                                                                  ║
    // ║  SECTIONS:                                                       ║
    // ║    1. Basic Drive (teleop)                                       ║
    // ║    2. Heading Control (pointing, aiming)                         ║
    // ║    3. Autonomous Movement (driveToPoint, moveThroughPose, etc.)  ║
    // ║    4. Utility (brake, resetPose)                                 ║
    // ╚══════════════════════════════════════════════════════════════════╝


    // ─────────────────────────────────────────────────────────────────
    // 1. BASIC DRIVE
    // ─────────────────────────────────────────────────────────────────

    /**
     * Default teleop drive command.
     * Accepts a ChassisVelocities supplier so Robot.java can compute speeds
     * from controller inputs each loop iteration.
     *
     *   swerve.setDefaultCommand(swerve.driveCommand(() -> computeSpeeds()));
     */
    public Command driveCommand(Supplier<ChassisVelocities> speeds) {
        return run(() -> drive(speeds.get()))
            .finallyDo(interrupted -> stop())
            .withName("Swerve.drive");
    }

    /**
     * Drives at fixed ChassisVelocities. Used by auto commands.
     */
    public Command driveCommand(ChassisVelocities speeds) {
        return run(() -> drive(speeds))
            .finallyDo(interrupted -> stop())
            .withName("Swerve.driveFixed");
    }

    /**
     * Drives at the given ChassisVelocities for a fixed number of seconds, then stops.
     * Called by AutoBuilder.driveForTime().
     */
    public Command driveForTime(ChassisVelocities speeds, double seconds) {
        return run(() -> drive(speeds))
            .withTimeout(seconds)
            .finallyDo(interrupted -> stop())
            .withName("Swerve.driveForTime");
    }

    /** Stops all modules. Instantaneous (runOnce). */
    public Command stopCommand() {
        return runOnce(this::stop)
            .withName("Swerve.stop");
    }

    /**
     * Zeroes the gyro so the robot's current facing direction becomes field-forward (0°).
     * Bind to a controller button (left bumper is standard) and press while the robot
     * is physically pointing toward the opposing alliance wall.
     */
    public Command zeroHeadingCommand() {
        return runOnce(this::zeroHeading)
            .withName("Swerve.zeroHeading");
    }

    /** Resets the robot's pose estimate to a given field position. */
    public Command resetPoseCommand(Pose2d pose) {
        return runOnce(() -> resetPose(pose))
            .withName("Swerve.resetPose");
    }


    // ─────────────────────────────────────────────────────────────────
    // 2. HEADING CONTROL
    // ─────────────────────────────────────────────────────────────────

    /**
     * Rotates the robot in place until it faces a fixed field position.
     *
     * The robot looks toward the XY of targetPose — the target's own rotation
     * is ignored. Finishes within HEADING_TOLERANCE_DEG (5°).
     *
     * Use case: snap to face the speaker/hub before shooting.
     *
     * @param targetPose  the field position to face toward
     */
    public Command pointToTarget(Pose2d targetPose) {
        return run(() -> {
            Pose2d current     = getPose();
            Rotation2d desired = computeHeadingToTarget(current, targetPose);
            drive(new ChassisVelocities(0.0, 0.0,
                computeOmega(current.getRotation(), desired)));
        })
        .until(() -> {
            Pose2d current     = getPose();
            Rotation2d desired = computeHeadingToTarget(current, targetPose);
            return Math.abs(MathUtil.angleModulus(
                current.getRotation().minus(desired).getRadians()
            )) < Math.toRadians(HEADING_TOLERANCE_DEG);
        })
        .finallyDo(interrupted -> stop())
        .withName("Swerve.pointToTarget");
    }

    /**
     * Rotates the robot in place to continuously face a dynamic target.
     * The target pose is re-read from the supplier every loop.
     *
     * Use case: continuously track a moving vision target or live shooter setpoint.
     *
     * @param targetPoseSupplier  supplier that returns the current target pose
     */
    public Command pointToTarget(Supplier<Pose2d> targetPoseSupplier) {
        return run(() -> {
            Pose2d current     = getPose();
            Rotation2d desired = computeHeadingToTarget(current, targetPoseSupplier.get());
            drive(new ChassisVelocities(0.0, 0.0,
                computeOmega(current.getRotation(), desired)));
        })
        .until(() -> {
            Pose2d current     = getPose();
            Rotation2d desired = computeHeadingToTarget(current, targetPoseSupplier.get());
            return Math.abs(MathUtil.angleModulus(
                current.getRotation().minus(desired).getRadians()
            )) < Math.toRadians(HEADING_TOLERANCE_DEG);
        })
        .finallyDo(interrupted -> stop())
        .withName("Swerve.pointToTargetDynamic");
    }

    /**
     * Rotates the robot in place to match a specific heading in degrees.
     *
     * Finishes within HEADING_TOLERANCE_DEG (5°).
     *
     * @param angleDeg  desired heading in degrees (e.g. 90 = left, 180 = backwards)
     */
    public Command changeHeading(double angleDeg) {
        return changeHeading(Rotation2d.fromDegrees(angleDeg));
    }

    /**
     * Rotates the robot in place to match the given Rotation2d.
     *
     * Differs from pointToTarget: this snaps to a specific angle,
     * not a direction toward a field location. Useful for correcting heading
     * drift after moveThroughPose, or for aligning with a wall.
     *
     * Finishes within HEADING_TOLERANCE_DEG (5°).
     *
     * @param targetHeading  the desired heading
     */
    public Command changeHeading(Rotation2d targetHeading) {
        return run(() -> {
            double omega = computeOmega(getPose().getRotation(), targetHeading);
            drive(new ChassisVelocities(0.0, 0.0, omega));
        })
        .until(() ->
            Math.abs(MathUtil.angleModulus(
                getPose().getRotation().minus(targetHeading).getRadians()
            )) < Math.toRadians(HEADING_TOLERANCE_DEG)
        )
        .finallyDo(interrupted -> stop())
        .withName("Swerve.changeHeading");
    }

    /**
     * Rotates the robot in place to match the heading stored in a target Pose2d,
     * ignoring that pose's XY position.
     *
     * @param targetPose  the pose whose .getRotation() defines the desired heading
     */
    public Command changeHeading(Pose2d targetPose) {
        return changeHeading(targetPose.getRotation());
    }

    /**
     * Drives with driver-supplied translation while automatically aiming at a target.
     *
     * Includes motion compensation — the aim leads the robot's velocity so projectiles
     * arrive correctly even while moving. Tune AIM_LEAD_TIME at the top of this file.
     *
     * This command NEVER finishes on its own. Bind with whileTrue() so it cancels
     * when the button is released, returning control to the default drive command.
     *
     * Use case: driver holds a button to auto-aim while keeping full translation control.
     *
     * @param targetPoseSupplier  field target to aim at (re-read every loop)
     * @param xSupplier           driver forward/back input (m/s, field-relative)
     * @param ySupplier           driver strafe input (m/s, field-relative)
     */
    public Command maintainHeadingToTarget(
        Supplier<Pose2d> targetPoseSupplier,
        Supplier<Double> xSupplier,
        Supplier<Double> ySupplier
    ) {
        return run(() -> {
            Pose2d currentPose  = getPose();
            Pose2d targetPose   = targetPoseSupplier.get();
            ChassisVelocities field = getFieldRelativeSpeeds();

            // Shift the aim point forward in time to compensate for robot motion.
            // If the robot moves right at 2 m/s and lead time is 0.25 s,
            // the compensated target shifts 0.5 m right — correcting for
            // the time the game piece takes to travel to the target.
            Translation2d compensated = targetPose.getTranslation().minus(
                new Translation2d(
                    field.vx * AIM_LEAD_TIME,
                    field.vy * AIM_LEAD_TIME
                )
            );

            Rotation2d desiredHeading = compensated
                .minus(currentPose.getTranslation())
                .getAngle()
                // 2027: getAngle() is empty for a zero-length vector (robot exactly
                // on the target). Keep the current heading in that case.
                .orElse(currentPose.getRotation());

            double omega = computeOmega(currentPose.getRotation(), desiredHeading);
            // xSupplier / ySupplier are field-relative — convert to robot-relative
            // before passing to drive() so translation is correct at any heading.
            drive(new ChassisVelocities(
                xSupplier.get(), ySupplier.get(), omega
            ).toRobotRelative(currentPose.getRotation()));
        })
        .finallyDo(interrupted -> stop())
        .withName("Swerve.maintainHeadingToTarget");
    }


    // ─────────────────────────────────────────────────────────────────
    // 3. AUTONOMOUS MOVEMENT
    // ─────────────────────────────────────────────────────────────────

    /**
     * Drives toward a target pose at a fixed velocity and stops when within
     * ARRIVE_THRESHOLD (5 cm) of the target XY.
     *
     * Rotation toward the target pose's heading is corrected proportionally
     * throughout the move.
     *
     * @param targetPose  the field pose to drive to
     * @param velocity    translation speed (m/s)
     * @param stopAtEnd   true = stop when done; false = leave velocity applied (for chaining)
     */
    public Command driveToPoint(Pose2d targetPose, double velocity, boolean stopAtEnd) {
        // stall[0,1] = last XY where movement was detected; stall[2] = timestamp
        final double[] stall = new double[3];

        return runOnce(() -> {
            Pose2d p = getPose();
            stall[0] = p.getX();
            stall[1] = p.getY();
            stall[2] = Timer.getTimestamp();
        })
        .andThen(run(() -> {
            Pose2d current     = getPose();
            Rotation2d heading = current.getRotation();

            double dx       = targetPose.getX() - current.getX();
            double dy       = targetPose.getY() - current.getY();
            double distance = Math.hypot(dx, dy);

            double apfSpeed = Math.min(ARRIVE_KP * distance, velocity);
            double ux = (distance > ARRIVE_THRESHOLD) ? dx / distance : 0.0;
            double uy = (distance > ARRIVE_THRESHOLD) ? dy / distance : 0.0;

            driveRobotRelative(new ChassisVelocities(
                ux * apfSpeed, uy * apfSpeed,
                computeOmega(heading, targetPose.getRotation())
            ).toRobotRelative(heading));

            // Advance stall checkpoint whenever the robot makes meaningful progress
            double moveDx = Math.abs(current.getX() - stall[0]);
            double moveDy = Math.abs(current.getY() - stall[1]);
            if (moveDx > STALL_THRESHOLD || moveDy > STALL_THRESHOLD) {
                stall[0] = current.getX();
                stall[1] = current.getY();
                stall[2] = Timer.getTimestamp();
            }
        }))
        .until(() -> {
            Pose2d current = getPose();
            if (current.getTranslation().getDistance(targetPose.getTranslation()) < ARRIVE_THRESHOLD)
                return true;
            double dx = Math.abs(current.getX() - stall[0]);
            double dy = Math.abs(current.getY() - stall[1]);
            return dx < STALL_THRESHOLD
                && dy < STALL_THRESHOLD
                && (Timer.getTimestamp() - stall[2]) > STALL_TIMEOUT;
        })
        .finallyDo(interrupted -> { if (stopAtEnd) stop(); })
        .withName("Swerve.driveToPoint");
    }

    /**
     * Drives through a waypoint at constant speed without slowing down.
     *
     * Unlike driveToPoint, the command finishes as soon as the robot enters the
     * pass radius — there is no deceleration. This allows smooth, high-speed
     * paths through multiple chained waypoints.
     *
     * Rotation is controlled by a PID toward the target pose's heading.
     * Tune THETA_KP / KI / KD and MOVE_THROUGH_DEFAULT_RADIUS at the top of this file.
     *
     * Stall detection prevents the robot from getting permanently stuck if
     * it hits an obstacle — the command auto-exits after STALL_TIMEOUT seconds
     * of insufficient movement.
     *
     * @param targetPose  the waypoint pose to pass through
     * @param maxSpeed    translation speed (m/s)
     * @param maxAngular  maximum rotation speed (rad/s), clamps PID output
     * @param passRadius  distance to consider the waypoint "passed" (meters)
     */
    @SuppressWarnings("resource")
    public Command moveThroughPose(
        Pose2d targetPose,
        double maxSpeed,
        double maxAngular,
        double passRadius
    ) {
        PIDController thetaPID = new PIDController(THETA_KP, THETA_KI, THETA_KD);
        thetaPID.enableContinuousInput(-Math.PI, Math.PI);

        // stall[0,1] = last XY where movement was detected; stall[2] = stall timestamp; stall[3] = command start
        final double[] stall = new double[4];

        return runOnce(() -> {
            Pose2d p = getPose();
            stall[0] = p.getX();
            stall[1] = p.getY();
            stall[2] = Timer.getTimestamp();
            stall[3] = stall[2]; // wall-clock start for hard deadline
            thetaPID.reset();
        })
        .andThen(run(() -> {
            Pose2d current = getPose();

            // Normalized direction vector toward waypoint
            double dx       = targetPose.getX() - current.getX();
            double dy       = targetPose.getY() - current.getY();
            double distance = Math.hypot(dx, dy);
            double dirX     = dx / (distance + 1e-9);
            double dirY     = dy / (distance + 1e-9);

            // PID rotation toward target heading, clamped to maxAngular
            double omega = Math.clamp(
                thetaPID.calculate(
                    current.getRotation().getRadians(),
                    targetPose.getRotation().getRadians()
                ),
                -maxAngular, maxAngular
            );

            driveRobotRelative(new ChassisVelocities(
                dirX * maxSpeed, dirY * maxSpeed, omega
            ).toRobotRelative(current.getRotation()));

            // Update stall tracker if the robot is actually moving
            double moveDx = Math.abs(current.getX() - stall[0]);
            double moveDy = Math.abs(current.getY() - stall[1]);
            if (moveDx > STALL_THRESHOLD || moveDy > STALL_THRESHOLD) {
                stall[0] = current.getX();
                stall[1] = current.getY();
                stall[2] = Timer.getTimestamp();
            }
        }))
        .until(() -> {
            Pose2d current = getPose();

            // Done: robot entered pass radius
            if (current.getTranslation().getDistance(targetPose.getTranslation()) < passRadius)
                return true;

            // Done: stall timeout expired (robot not moving)
            double dx = Math.abs(current.getX() - stall[0]);
            double dy = Math.abs(current.getY() - stall[1]);
            if (dx < STALL_THRESHOLD && dy < STALL_THRESHOLD
                    && (Timer.getTimestamp() - stall[2]) > STALL_TIMEOUT)
                return true;

            // Done: hard wall-clock deadline — catches oscillation that keeps resetting the stall timer
            return (Timer.getTimestamp() - stall[3]) > MAX_MOVETHROUGH_SECONDS;
        })
        .finallyDo(interrupted -> stop())
        .withName("Swerve.moveThroughPose");
    }

    /**
     * Convenience overload of moveThroughPose using the default pass radius.
     * Use this when you don't need to tune the radius per waypoint.
     */
    public Command moveThroughPose(Pose2d targetPose, double maxSpeed, double maxAngular) {
        return moveThroughPose(targetPose, maxSpeed, maxAngular, MOVE_THROUGH_DEFAULT_RADIUS);
    }

    /**
     * Drives forward a fixed distance along the robot's current heading.
     *
     * The target pose is computed from the robot's starting position and heading
     * at the moment the command initializes. Heading is held with proportional
     * correction throughout. Finishes within ARRIVE_THRESHOLD (5 cm).
     *
     * Use case: fallback auto when vision is offline — just drive a known distance.
     *
     * @param distanceMeters  distance to drive (meters, positive = forward)
     * @param velocity        translation speed (m/s)
     * @param stopAtEnd       true = stop when done
     */
    public Command driveForwardMeters(double distanceMeters, double velocity, boolean stopAtEnd) {
        // Single-element array so the target pose computed in runOnce is accessible
        // in the run() and until() closures without subsystem-level mutable state.
        final Pose2d[] target = { new Pose2d() };

        return runOnce(() -> {
            Pose2d current = getPose();
            Rotation2d hdg = current.getRotation();
            target[0] = new Pose2d(
                current.getX() + distanceMeters * hdg.getCos(),
                current.getY() + distanceMeters * hdg.getSin(),
                hdg
            );
        })
        .andThen(run(() -> {
            Pose2d current = getPose();
            Rotation2d hdg = current.getRotation();

            double dx       = target[0].getX() - current.getX();
            double dy       = target[0].getY() - current.getY();
            double distance = Math.hypot(dx, dy);

            double apfSpeed = Math.min(ARRIVE_KP * distance, velocity);
            double ux = (distance > ARRIVE_THRESHOLD) ? dx / distance : 0.0;
            double uy = (distance > ARRIVE_THRESHOLD) ? dy / distance : 0.0;

            driveRobotRelative(new ChassisVelocities(
                ux * apfSpeed, uy * apfSpeed,
                computeOmega(hdg, target[0].getRotation())
            ).toRobotRelative(hdg));
        }))
        .until(() ->
            getPose().getTranslation().getDistance(target[0].getTranslation()) < ARRIVE_THRESHOLD
        )
        .finallyDo(interrupted -> { if (stopAtEnd) stop(); })
        .withName("Swerve.driveForwardMeters");
    }


    // ─────────────────────────────────────────────────────────────────
    // 4. UTILITY
    // ─────────────────────────────────────────────────────────────────

    /**
     * Brakes all modules in the X configuration to resist pushing.
     *
     * FL and BR point to +45°, FR and BL point to -45°. The crossed pattern
     * makes it very hard to push the robot out of position.
     *
     * The command runs continuously and must be interrupted to exit.
     * Bind with whileTrue() so the brake releases when the button is released.
     *
     * Use case: hold position while shooting, resist defense, stay on a slope.
     */
    public Command brakeCommand() {
        return run(this::brake)
            .finallyDo(interrupted -> stop())
            .withName("Swerve.brake");
    }
}
