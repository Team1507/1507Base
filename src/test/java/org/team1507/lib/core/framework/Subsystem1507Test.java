//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.framework;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.CANBus;

import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.hal.HAL;

import org.team1507.lib.core.impl.ctre.Motor1507;
import org.team1507.lib.core.util.MotorConfig;

// ─────────────────────────────────────────────────────────────────────────────
// Subsystem1507Test
//
// Checks what Subsystem1507 does for every subsystem automatically, so student
// subsystems don't have to:
//   - motors made with motor(...) are refreshed and simulated every loop
//     (the test subsystem below NEVER calls refresh() or simulationPeriodic())
//   - motor telemetry nests under the subsystem name
//   - total supply current adds up its motors
//   - warnIf() alerts turn on and off with their condition
//   - a missing default CAN bus explains the fix
// ─────────────────────────────────────────────────────────────────────────────
class Subsystem1507Test {

    private static final CANBus BUS = new CANBus(CANPort.CAN_S0);
    private static final Scheduler scheduler = Scheduler.getDefault();

    /** A subsystem written the way students should: no refresh, no sim code. */
    private static final class TestArm extends Subsystem1507 {
        final Motor1507 arm;
        boolean problem = false;

        TestArm(int canId) {
            super("TestArm" + canId);
            arm = motor("Arm", Motor1507.Type.FX, canId,
                MotorConfig.arm().gearRatio(50).withSimVelocityRps(1.0).build());
        }

        @Override
        public void periodic() {
            warnIf(problem, "Something is wrong");
        }
    }

    @BeforeAll
    static void setUp() {
        HAL.initialize();
        Subsystem1507.setDefaultCanBus(BUS);
    }

    private static void runLoops(int loops) {
        for (int i = 0; i < loops; i++) {
            scheduler.run();
        }
    }

    @Test
    void motorsAreSimulatedWithoutAnyCodeInTheSubsystem() {
        TestArm subsystem = new TestArm(40);
        subsystem.arm.setPosition(90);

        runLoops(50);   // 1 s at 1 rotation/s: more than a quarter turn

        assertEquals(90.0, subsystem.arm.getPosition(), 1e-6);
        assertTrue(subsystem.arm.isAtTarget());
    }

    @Test
    void motorTelemetryNestsUnderTheSubsystem() {
        TestArm subsystem = new TestArm(41);
        assertEquals("TestArm41/Arm", subsystem.arm.getName());
    }

    @Test
    void totalSupplyCurrentAddsUpItsMotors() {
        TestArm subsystem = new TestArm(42);
        runLoops(2);
        assertEquals(subsystem.arm.getSupplyCurrent(), subsystem.getSupplyCurrent(), 1e-9);
    }

    @Test
    void warnIfAlertFollowsTheCondition() {
        TestArm subsystem = new TestArm(43);

        runLoops(1);
        assertFalse(subsystem.isAlertActive("Something is wrong"));

        subsystem.problem = true;
        runLoops(1);
        assertTrue(subsystem.isAlertActive("Something is wrong"));

        subsystem.problem = false;
        runLoops(1);
        assertFalse(subsystem.isAlertActive("Something is wrong"), "alert should clear itself");
    }

    @Test
    void missingDefaultCanBusExplainsTheFix() {
        Subsystem1507.setDefaultCanBus(null);
        try {
            IllegalStateException e = assertThrows(IllegalStateException.class, () -> new TestArm(44));
            assertTrue(e.getMessage().contains("setDefaultCanBus"));
        } finally {
            Subsystem1507.setDefaultCanBus(BUS);
        }
    }
}
