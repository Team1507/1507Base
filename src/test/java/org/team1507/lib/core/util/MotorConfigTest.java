//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.function.Consumer;

import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import org.team1507.lib.core.impl.ctre.CtreMotorConfigurator;
import org.team1507.lib.core.util.MotorConfig.ControlMode;
import org.team1507.lib.core.util.MotorConfig.GravityType;

// ─────────────────────────────────────────────────────────────────────────────
// MotorConfigTest
//
// Checks the MotorConfig library itself: presets, overrides, copy(), and what
// each config turns into on the CTRE side. Student configs in Constants.java
// are checked separately by MotorConfigConstantsTest.
// ─────────────────────────────────────────────────────────────────────────────
class MotorConfigTest {

    /** Every preset, filled in the minimum way a student would. */
    private static List<MotorConfig> allPresets() {
        return List.of(
            MotorConfig.roller().build(),
            MotorConfig.flywheel().build(),
            MotorConfig.arm().gearRatio(50).build(),
            MotorConfig.elevator().withDrumDiameterMeters(0.05).build(),
            MotorConfig.position().build()
        );
    }

    // ── Presets ─────────────────────────────────────────────────────────────

    @Test
    void everyPresetIsValidAndHasASupplyLimit() {
        for (MotorConfig config : allPresets()) {
            assertEquals(List.of(), config.problems(), config.preset() + " preset has problems");
            assertTrue(config.supplyCurrentLimit().baseUnitMagnitude() <= 40.0,
                config.preset() + " preset should protect the battery (supply limit <= 40 A)");
        }
    }

    @Test
    void presetStartingValues() {
        MotorConfig roller = MotorConfig.roller().build();
        assertEquals(ControlMode.TORQUE, roller.mode());
        assertFalse(roller.brakeMode(), "rollers coast by default");
        assertEquals(15.0, roller.supplyLowerLimitAmps());

        MotorConfig flywheel = MotorConfig.flywheel().build();
        assertEquals(ControlMode.VELOCITY, flywheel.mode());
        assertFalse(flywheel.brakeMode());

        MotorConfig arm = MotorConfig.arm().build();
        assertEquals(ControlMode.POSITION, arm.mode());
        assertTrue(arm.brakeMode());
        assertEquals(GravityType.COSINE, arm.gravityType());

        MotorConfig elevator = MotorConfig.elevator().withDrumDiameterMeters(0.05).build();
        assertEquals(GravityType.CONSTANT, elevator.gravityType());
        assertEquals(Math.PI * 0.05, elevator.drumCircumferenceMeters(), 1e-12);
    }

    // ── Overrides ───────────────────────────────────────────────────────────

    @Test
    void anythingAfterThePresetWins() {
        MotorConfig plain = MotorConfig.arm().build();
        assertFalse(plain.motorInverted());
        assertTrue(plain.brakeMode());

        MotorConfig custom = MotorConfig.arm().inverted(true).withCoast().build();
        assertTrue(custom.motorInverted());
        assertFalse(custom.brakeMode());
    }

    @Test
    void withGravityUsesThePresetsGravityType() {
        assertEquals(GravityType.COSINE, MotorConfig.arm().withGravity(0.1).build().gravityType());
        assertEquals(0.1, MotorConfig.arm().withGravity(0.1).build().kG());
        assertThrows(IllegalStateException.class, () -> MotorConfig.flywheel().withGravity(0.1));
    }

    @Test
    void inchesConvertToMeters() {
        MotorConfig inches = MotorConfig.elevator().withDrumDiameterInches(2.0).withToleranceInches(0.5).build();
        assertEquals(Math.PI * 2.0 * 0.0254, inches.drumCircumferenceMeters(), 1e-12);
        assertEquals(0.5 * 0.0254, inches.toleranceMeters(), 1e-12);
    }

    // ── copy() ──────────────────────────────────────────────────────────────

    @Test
    void copyKeepsEveryField() {
        // Every field set to a non-default value. If copy() ever forgets a field,
        // that field reverts to its default and this comparison fails.
        Consumer<TalonFXConfiguration> fx = cfg -> {};
        Consumer<TalonFXSConfiguration> fxs = cfg -> {};
        MotorConfig everything = MotorConfig.arm()
            .slot(1)
            .mode(ControlMode.MOTION_MAGIC)
            .inverted(true)
            .withPID(1, 2, 3)
            .withFeedforward(4, 5, 6)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
            .withGravity(7, GravityType.CONSTANT)
            .withVoltageLimits(10, -9)
            .withStatorCurrentLimit(55)
            .withSupplyCurrentLimit(25)
            .withSupplyLowerLimit(12, 3)
            .withPeakTorqueCurrent(33, -22)
            .withForwardLimit(true, true, 1.5)
            .forwardLimitType(ForwardLimitTypeValue.NormallyClosed)
            .withReverseLimit(true, true, -1.5)
            .reverseLimitType(ReverseLimitTypeValue.NormallyClosed)
            .withFeedbackSensor(FeedbackSensorSourceValue.RemoteCANcoder)
            .withRemoteSensorId(42)
            .withRotorToSensorRatio(3)
            .gearRatio(17)
            .withRotorOffset(0.25)
            .withDrumDiameterMeters(0.04)
            .withToleranceDegrees(1.5)
            .withToleranceMeters(0.02)
            .withToleranceRPM(75)
            .withStallCurrentThreshold(44)
            .withStallVelocityThreshold(0.7)
            .withStallTime(0.9)
            .withCoast()
            .withContinuousWrap()
            .withFOC()
            .withSimVelocityRps(12)
            .withCtreConfig(fx)
            .withCtreFxsConfig(fxs)
            .build();

        assertEquals(everything, MotorConfig.copy(everything).build());
    }

    @Test
    void copyThenInvertChangesOnlyInversion() {
        MotorConfig right = MotorConfig.flywheel().withPID(0.3, 0, 0).withFeedforward(0, 0.12, 0).build();
        MotorConfig left = MotorConfig.copy(right).inverted(true).build();

        assertTrue(left.motorInverted());
        assertNotEquals(right, left);
        assertEquals(right, MotorConfig.copy(left).inverted(false).build());
    }

    // ── What it becomes on the CTRE side ────────────────────────────────────

    @Test
    void armBecomesTheRightCtreConfig() {
        TalonFXConfiguration cfg = CtreMotorConfigurator.toTalonFXConfiguration(
            MotorConfig.arm().gearRatio(50).withPID(0.5, 0, 0).withGravity(0.1).inverted(true).build());

        assertEquals(50.0, cfg.Feedback.SensorToMechanismRatio);
        assertEquals(NeutralModeValue.Brake, cfg.MotorOutput.NeutralMode);
        assertEquals(InvertedValue.Clockwise_Positive, cfg.MotorOutput.Inverted);
        assertEquals(GravityTypeValue.Arm_Cosine, cfg.Slot0.GravityType);
        assertEquals(0.1, cfg.Slot0.kG);
        assertEquals(0.5, cfg.Slot0.kP);
        assertEquals(60.0, cfg.CurrentLimits.StatorCurrentLimit);
        assertEquals(30.0, cfg.CurrentLimits.SupplyCurrentLimit);
        assertTrue(cfg.CurrentLimits.SupplyCurrentLimitEnable);
    }

    @Test
    void rollerGetsTheSustainedSupplyLimit() {
        TalonFXConfiguration cfg = CtreMotorConfigurator.toTalonFXConfiguration(MotorConfig.roller().build());
        assertEquals(35.0, cfg.CurrentLimits.SupplyCurrentLimit);
        assertEquals(15.0, cfg.CurrentLimits.SupplyCurrentLowerLimit);
        assertEquals(4.0, cfg.CurrentLimits.SupplyCurrentLowerTime);
        assertEquals(NeutralModeValue.Coast, cfg.MotorOutput.NeutralMode);
    }

    @Test
    void ctreOverrideRunsLast() {
        TalonFXConfiguration cfg = CtreMotorConfigurator.toTalonFXConfiguration(
            MotorConfig.arm()
                .withStatorCurrentLimit(60)
                .withCtreConfig(c -> c.CurrentLimits.StatorCurrentLimit = 99)
                .build());
        assertEquals(99.0, cfg.CurrentLimits.StatorCurrentLimit);
    }

    // ── problems() catches mistakes ──────────────────────────────────────────

    @Test
    void problemsCatchCommonMistakes() {
        assertFalse(MotorConfig.elevator().build().problems().isEmpty(),
            "elevator without a drum diameter");
        assertFalse(MotorConfig.roller().withPID(1, 0, 0).build().problems().isEmpty(),
            "gains on an open-loop roller are ignored");
        assertFalse(MotorConfig.builder(ControlMode.TORQUE).build().problems().isEmpty(),
            "torque control without FOC");
        assertFalse(MotorConfig.flywheel().withSupplyLowerLimit(60, 1).build().problems().isEmpty(),
            "lower limit above the supply limit");
        assertFalse(MotorConfig.flywheel().withGravity(0.1, GravityType.COSINE).build().problems().isEmpty(),
            "gravity compensation on velocity control");
    }
}
