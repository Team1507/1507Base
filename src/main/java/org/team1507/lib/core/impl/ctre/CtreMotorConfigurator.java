//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.impl.ctre;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.*;

import org.team1507.lib.core.util.MotorConfig;
import org.team1507.lib.core.util.MotorConfig.ControlMode;
import org.team1507.lib.core.util.MotorConfig.GravityType;

/**
 * Applies {@link MotorConfig} specifications to CTRE motor controllers.
 *
 * <p>This class translates declarative motor configuration into CTRE Phoenix 6
 * configuration objects. It supports both {@link TalonFX} and {@link TalonFXS}
 * motors and applies all configuration in a single deterministic pass.
 */
public final class CtreMotorConfigurator {

    private CtreMotorConfigurator() {}

    /**
     * Applies one or more {@link MotorConfig} objects to a CTRE motor controller.
     *
     * <p>The first configuration (slot 0) defines base motor behavior such as
     * inversion, neutral mode, current limits, voltage limits, limit switches,
     * and feedback sensor settings. Additional configurations populate PID and
     * feedforward gains for other control slots.
     *
     * @param motor   a {@link TalonFX} or {@link TalonFXS} instance
     * @param configs ordered motor configuration specifications
     */
    public static void apply(Object motor, MotorConfig... configs) {
        if (configs.length == 0 || configs[0].slotNumber() != 0) {
            throw new IllegalArgumentException("First MotorConfig must be slot 0");
        }

        if (motor instanceof TalonFX fx) {
            applyFX(fx, configs);
        } else if (motor instanceof TalonFXS fxs) {
            applyFXS(fxs, configs);
        } else {
            throw new IllegalArgumentException("Unsupported motor type");
        }
    }

    // ============================================================
    // TALON FX
    // ============================================================

    /**
     * Applies all configuration slots and base settings to a {@link TalonFX}.
     *
     * <p>This method builds a complete {@link TalonFXConfiguration}, populates
     * all PID slots, applies base motor settings, and commits the configuration
     * to hardware in a single update.
     *
     * @param motor   the TalonFX instance to configure
     * @param configs ordered motor configuration specifications
     */
    private static void applyFX(TalonFX motor, MotorConfig[] configs) {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        for (MotorConfig config : configs) {
            applySlot(cfg, config);
        }

        applyBase(cfg, configs[0]);
        safeApply(motor, cfg);
    }

    // ============================================================
    // TALON FXS (MINION)
    // ============================================================

    /**
     * Applies all configuration slots and base settings to a {@link TalonFXS}.
     *
     * <p>This method builds a complete {@link TalonFXSConfiguration}, applies
     * Minion-specific commutation settings, populates PID slots, applies base
     * motor settings, and commits the configuration to hardware.
     *
     * @param motor   the TalonFXS instance to configure
     * @param configs ordered motor configuration specifications
     */
    private static void applyFXS(TalonFXS motor, MotorConfig[] configs) {
        TalonFXSConfiguration cfg = new TalonFXSConfiguration();

        cfg.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        for (MotorConfig config : configs) {
            applySlot(cfg, config);
        }

        applyBase(cfg, configs[0]);
        safeApply(motor, cfg);
    }

    // ============================================================
    // SLOT ROUTING
    // ============================================================

    /**
     * Routes a {@link MotorConfig} to the appropriate PID slot of a
     * {@link TalonFXConfiguration}.
     *
     * @param cfg     the TalonFX configuration being populated
     * @param config  the motor configuration for a specific slot
     */
    private static void applySlot(TalonFXConfiguration cfg, MotorConfig config) {
        switch (config.slotNumber()) {
            case 1 -> applyToSlot(cfg.Slot1, config);
            case 2 -> applyToSlot(cfg.Slot2, config);
            default -> applyToSlot(cfg.Slot0, config);
        }
    }

    /**
     * Routes a {@link MotorConfig} to the appropriate PID slot of a
     * {@link TalonFXSConfiguration}.
     *
     * @param cfg     the TalonFXS configuration being populated
     * @param config  the motor configuration for a specific slot
     */
    private static void applySlot(TalonFXSConfiguration cfg, MotorConfig config) {
        switch (config.slotNumber()) {
            case 1 -> applyToSlot(cfg.Slot1, config);
            case 2 -> applyToSlot(cfg.Slot2, config);
            default -> applyToSlot(cfg.Slot0, config);
        }
    }

    // ============================================================
    // SLOT CONFIGURATION
    //
    // Slot0Configs / Slot1Configs / Slot2Configs do not share a common
    // writable interface in Phoenix 6, so gains must be applied via three
    // separate overloads with identical bodies. Keep them in sync.
    // ============================================================

    private static void applyToSlot(Slot0Configs slot, MotorConfig config) { applyGains(slot, config); }
    private static void applyToSlot(Slot1Configs slot, MotorConfig config) { applyGains(slot, config); }
    private static void applyToSlot(Slot2Configs slot, MotorConfig config) { applyGains(slot, config); }

    /**
     * Writes PID, feedforward, and gravity compensation gains into a slot.
     *
     * <p>Gains are skipped for {@code DUTY_CYCLE} and {@code TORQUE} modes —
     * both are open-loop and do not use slot-based closed-loop gains.
     *
     * @param slot    the slot configuration to modify
     * @param config  the motor configuration providing gain values
     */
    private static void applyGains(Slot0Configs slot, MotorConfig config) {
        if (config.mode() != ControlMode.DUTY_CYCLE && config.mode() != ControlMode.TORQUE) {
            slot.kP = config.kP();
            slot.kI = config.kI();
            slot.kD = config.kD();
            slot.kV = config.kV();
            slot.kS = config.kS();
            slot.kA = config.kA();
        }
        applyGravity(config, g -> slot.kG = g, t -> slot.GravityType = t);
    }

    /** @see #applyGains(Slot0Configs, MotorConfig) */
    private static void applyGains(Slot1Configs slot, MotorConfig config) {
        if (config.mode() != ControlMode.DUTY_CYCLE && config.mode() != ControlMode.TORQUE) {
            slot.kP = config.kP();
            slot.kI = config.kI();
            slot.kD = config.kD();
            slot.kV = config.kV();
            slot.kS = config.kS();
            slot.kA = config.kA();
        }
        applyGravity(config, g -> slot.kG = g, t -> slot.GravityType = t);
    }

    /** @see #applyGains(Slot0Configs, MotorConfig) */
    private static void applyGains(Slot2Configs slot, MotorConfig config) {
        if (config.mode() != ControlMode.DUTY_CYCLE && config.mode() != ControlMode.TORQUE) {
            slot.kP = config.kP();
            slot.kI = config.kI();
            slot.kD = config.kD();
            slot.kV = config.kV();
            slot.kS = config.kS();
            slot.kA = config.kA();
        }
        applyGravity(config, g -> slot.kG = g, t -> slot.GravityType = t);
    }

    /**
     * Applies gravity compensation to a slot.
     *
     * <p>Shared by all three slot overloads since the logic is identical but
     * the slot field references differ per type.
     *
     * @param config   motor configuration
     * @param setKg    setter for the slot's {@code kG} field
     * @param setType  setter for the slot's {@code GravityType} field
     */
    private static void applyGravity(
            MotorConfig config,
            java.util.function.DoubleConsumer setKg,
            java.util.function.Consumer<GravityTypeValue> setType) {
        if (config.gravityType() == GravityType.NONE) return;
        setKg.accept(config.kG());
        setType.accept(switch (config.gravityType()) {
            case COSINE   -> GravityTypeValue.Arm_Cosine;
            case CONSTANT -> GravityTypeValue.Elevator_Static;
            default       -> throw new IllegalArgumentException(
                "Unsupported GravityType: " + config.gravityType());
        });
    }

    // ============================================================
    // BASE CONFIGURATION (SLOT 0 AUTHORITY)
    // ============================================================

    /**
     * Applies base motor settings for a {@link TalonFX}, including inversion,
     * neutral mode, voltage limits, limit switches, current limits, and
     * feedback sensor configuration.
     *
     * @param cfg   the TalonFX configuration being populated
     * @param base  the slot 0 motor configuration
     */
    private static void applyBase(TalonFXConfiguration cfg, MotorConfig base) {
        applyMotorOutput(cfg.MotorOutput, base);
        applyVoltage(cfg.Voltage, base);
        applyLimitSwitches(cfg.HardwareLimitSwitch, base);
        applyCurrentLimits(cfg.CurrentLimits, base);
        applyTorqueCurrent(cfg.TorqueCurrent, base);
        applyFeedback(cfg.Feedback, base.feedback());

        cfg.ClosedLoopGeneral.ContinuousWrap = base.continuousWrap();
    }

    /**
     * Applies base motor settings for a {@link TalonFXS}, including inversion,
     * neutral mode, voltage limits, limit switches, current limits, and
     * external feedback sensor configuration.
     *
     * @param cfg   the TalonFXS configuration being populated
     * @param base  the slot 0 motor configuration
     */
    private static void applyBase(TalonFXSConfiguration cfg, MotorConfig base) {
        applyMotorOutput(cfg.MotorOutput, base);
        applyVoltage(cfg.Voltage, base);
        applyLimitSwitches(cfg.HardwareLimitSwitch, base);
        applyCurrentLimits(cfg.CurrentLimits, base);
        applyExternalFeedback(cfg.ExternalFeedback, base.feedback());

        cfg.ClosedLoopGeneral.ContinuousWrap = base.continuousWrap();
    }

    /**
     * Applies inversion and neutral mode settings.
     *
     * @param mo    motor output configuration
     * @param base  the slot 0 motor configuration
     */
    private static void applyMotorOutput(MotorOutputConfigs mo, MotorConfig base) {
        mo.Inverted = base.motorInverted()
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        mo.NeutralMode = base.brakeMode()
                ? NeutralModeValue.Brake
                : NeutralModeValue.Coast;
    }

    /**
     * Applies forward and reverse voltage limits.
     *
     * @param v     voltage configuration
     * @param base  the slot 0 motor configuration
     */
    private static void applyVoltage(VoltageConfigs v, MotorConfig base) {
        v.withPeakForwardVoltage(Volts.of(base.peakForwardVoltage()))
         .withPeakReverseVoltage(Volts.of(base.peakReverseVoltage()));
    }

    /**
     * Applies forward and reverse hardware limit switch settings.
     *
     * @param hw    limit switch configuration
     * @param base  the slot 0 motor configuration
     */
    private static void applyLimitSwitches(HardwareLimitSwitchConfigs hw, MotorConfig base) {
        hw.ForwardLimitEnable = base.forwardLimitEnable();
        hw.ForwardLimitAutosetPositionEnable = base.forwardLimitAutosetEnable();
        hw.ForwardLimitAutosetPositionValue = base.forwardLimitAutosetValue();
        hw.ForwardLimitType = base.forwardLimitType();

        hw.ReverseLimitEnable = base.reverseLimitEnable();
        hw.ReverseLimitAutosetPositionEnable = base.reverseLimitAutosetEnable();
        hw.ReverseLimitAutosetPositionValue = base.reverseLimitAutosetValue();
        hw.ReverseLimitType = base.reverseLimitType();
    }

    /**
     * Applies peak torque current limits for TorqueCurrentFOC mode (Phoenix Pro).
     *
     * <p>These limits are applied in addition to the stator current limit;
     * whichever is more restrictive takes effect. For non-torque motors the
     * defaults (800 A / -800 A) are effectively unlimited and have no impact.
     *
     * @param tc    torque current configuration
     * @param base  the slot 0 motor configuration
     */
    private static void applyTorqueCurrent(TorqueCurrentConfigs tc, MotorConfig base) {
        tc.PeakForwardTorqueCurrent = base.peakForwardTorqueCurrent();
        tc.PeakReverseTorqueCurrent = base.peakReverseTorqueCurrent();
    }

    /**
     * Applies stator and supply current limits.
     *
     * @param cl    current limit configuration
     * @param base  the slot 0 motor configuration
     */
    private static void applyCurrentLimits(CurrentLimitsConfigs cl, MotorConfig base) {
        cl.StatorCurrentLimit = base.statorCurrentLimit().in(Amps);
        cl.StatorCurrentLimitEnable = true;

        cl.SupplyCurrentLimit = base.supplyCurrentLimit().in(Amps);
        cl.SupplyCurrentLimitEnable = true;
    }

    // ============================================================
    // FEEDBACK CONFIGURATION
    // ============================================================

    /**
     * Applies feedback sensor configuration for a {@link TalonFX}, including
     * sensor source, remote device ID, gear ratios, and rotor offset.
     *
     * @param fb    feedback configuration block
     * @param cfg   feedback settings from {@link MotorConfig}
     */
    private static void applyFeedback(FeedbackConfigs fb, MotorConfig.Feedback cfg) {
        fb.FeedbackSensorSource = cfg.source();
        fb.FeedbackRemoteSensorID = cfg.remoteSensorId();
        fb.RotorToSensorRatio = cfg.rotorToSensorRatio();
        fb.SensorToMechanismRatio = cfg.sensorToMechanismRatio();
        fb.FeedbackRotorOffset = cfg.rotorOffset();
    }

    /**
     * Applies external feedback sensor configuration for a {@link TalonFXS},
     * including sensor source, remote device ID, gear ratios, and absolute
     * sensor offset.
     *
     * @param fb    external feedback configuration block
     * @param cfg   feedback settings from {@link MotorConfig}
     */
    private static void applyExternalFeedback(ExternalFeedbackConfigs fb, MotorConfig.Feedback cfg) {
        // RotorSensor means "use the internal encoder" — ExternalFeedbackSensorSourceValue
        // has no such constant; leave the default and only apply ratios.
        if (cfg.source() != FeedbackSensorSourceValue.RotorSensor) {
            fb.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.valueOf(cfg.source().name());
            fb.FeedbackRemoteSensorID = cfg.remoteSensorId();
            fb.RotorToSensorRatio = cfg.rotorToSensorRatio();
            fb.AbsoluteSensorOffset = cfg.rotorOffset();
        }
        fb.SensorToMechanismRatio = cfg.sensorToMechanismRatio();
    }

    // ============================================================
    // SAFE APPLY
    // ============================================================

    /**
     * Commits a {@link TalonFXConfiguration} to hardware and logs any
     * non‑successful status codes.
     *
     * @param motor  the TalonFX being configured
     * @param cfg    the configuration to apply
     */
    private static void safeApply(TalonFX motor, TalonFXConfiguration cfg) {
        var status = motor.getConfigurator().apply(cfg);
        if (!status.isOK()) {
            System.out.println("[WARN] Failed to apply config to FX "
                    + motor.getDeviceID() + ": " + status);
        }
    }

    /**
     * Commits a {@link TalonFXSConfiguration} to hardware and logs any
     * non‑successful status codes.
     *
     * @param motor  the TalonFXS being configured
     * @param cfg    the configuration to apply
     */
    private static void safeApply(TalonFXS motor, TalonFXSConfiguration cfg) {
        var status = motor.getConfigurator().apply(cfg);
        if (!status.isOK()) {
            System.out.println("[WARN] Failed to apply config to FXS "
                    + motor.getDeviceID() + ": " + status);
        }
    }
}
