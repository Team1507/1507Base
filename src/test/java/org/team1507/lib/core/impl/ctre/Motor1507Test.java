//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.impl.ctre;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.CANBus;

import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.hal.HAL;

import org.team1507.lib.core.impl.ctre.Motor1507.MotorSignal;
import org.team1507.lib.core.util.MotorConfig;

// ─────────────────────────────────────────────────────────────────────────────
// Motor1507Test
//
// Runs Motor1507 against CTRE's simulated TalonFX and checks the student-facing
// API: degrees, RPM, meters/inches, isAtTarget(), resetPosition(), get().
// Unit conversions are easy to get subtly wrong, so they are tested here.
// ─────────────────────────────────────────────────────────────────────────────
class Motor1507Test {

    private static final CANBus BUS = new CANBus(CANPort.CAN_S0);
    private static int nextId = 50;

    @BeforeAll
    static void initHal() {
        HAL.initialize();
    }

    private static Motor1507 motor(MotorConfig config) {
        return new Motor1507("Test/Motor" + nextId, Motor1507.Type.FX, nextId++, BUS, config);
    }

    /** Runs the simple simulation for a number of 20 ms loops. */
    private static void simulate(Motor1507 motor, int loops) {
        for (int i = 0; i < loops; i++) {
            motor.simulationPeriodic(0.02);
        }
    }

    @Test
    void positionIsInDegreesAndReachesTarget() {
        Motor1507 arm = motor(MotorConfig.arm().gearRatio(50).withSimVelocityRps(1.0).build());

        arm.setPosition(90);
        assertEquals(90.0, arm.getTargetPosition(), 1e-9);
        assertFalse(arm.isAtTarget(), "should not be there yet");

        simulate(arm, 50);   // 1 s at 1 rotation/s is more than a quarter turn
        assertEquals(90.0, arm.getPosition(), 1e-6);
        assertEquals(0.25, arm.getPositionRotations(), 1e-9);
        assertTrue(arm.isAtTarget());
        assertEquals(arm.getPosition(), arm.get(MotorSignal.POSITION), 1e-9);
    }

    @Test
    void isAtTargetUsesTheConfigTolerance() {
        Motor1507 arm = motor(MotorConfig.position().withToleranceDegrees(5).withSimVelocityRps(1.0).build());
        arm.resetPosition(0);
        arm.setPosition(4);   // within 5° of the start, so already "at target"
        assertTrue(arm.isAtTarget());
        arm.setPosition(10);  // 10° away: not at target
        assertFalse(arm.isAtTarget());
    }

    @Test
    void velocityIsInRpm() {
        Motor1507 shooter = motor(MotorConfig.flywheel().withSimVelocityRps(1000).build());

        shooter.setRPM(3000);
        assertEquals(3000.0, shooter.getTargetRPM(), 1e-9);
        simulate(shooter, 10);
        assertEquals(3000.0, shooter.getRPM(), 1e-6);
        assertEquals(50.0, shooter.getRPS(), 1e-6);
        assertTrue(shooter.isAtTarget());

        shooter.setRPS(10);
        assertEquals(600.0, shooter.getTargetRPM(), 1e-9);
    }

    @Test
    void elevatorWorksInMetersAndInches() {
        Motor1507 elevator = motor(MotorConfig.elevator()
            .withDrumDiameterMeters(0.05)
            .withSimVelocityRps(100)
            .build());

        elevator.setMeters(0.5);
        simulate(elevator, 50);
        assertEquals(0.5, elevator.getMeters(), 1e-6);
        assertEquals(0.5 / 0.0254, elevator.getInches(), 1e-6);
        assertTrue(elevator.isAtTarget());

        elevator.setInches(10);
        simulate(elevator, 50);
        assertEquals(10.0, elevator.getInches(), 1e-6);
    }

    @Test
    void metersWithoutADrumExplainTheFix() {
        Motor1507 arm = motor(MotorConfig.arm().build());
        IllegalStateException e = assertThrows(IllegalStateException.class, () -> arm.setMeters(1.0));
        assertTrue(e.getMessage().contains("withDrumDiameter"));
    }

    @Test
    void resetPositionSetsWhereTheMotorIs() {
        Motor1507 hopper = motor(MotorConfig.position().build());
        hopper.resetPosition(45);
        assertEquals(45.0, hopper.getPosition(), 1e-9);
    }

    @Test
    void openLoopHasNoTarget() {
        Motor1507 roller = motor(MotorConfig.roller().build());
        roller.runCurrent(20);
        assertFalse(roller.isAtTarget());
        assertTrue(Double.isNaN(roller.getTargetPosition()));
        assertTrue(Double.isNaN(roller.getTargetRPM()));
        roller.stop();
        assertFalse(roller.isAtTarget());
    }

    @Test
    void everySignalCanBeRead() {
        Motor1507 motor = motor(MotorConfig.flywheel().build());
        for (MotorSignal signal : MotorSignal.values()) {
            double value = motor.get(signal);
            assertFalse(Double.isNaN(value), signal + " returned NaN");
        }
    }
}
