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
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.CANBus;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.telemetry.MockTelemetryBackend;
import org.wpilib.telemetry.TelemetryRegistry;

import org.team1507.lib.core.impl.ctre.Motor1507;
import org.team1507.lib.core.logging.CommandLog;
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
//   - every motor signal is logged every loop, with no logging code in the subsystem
//   - the command using a subsystem is logged (CommandLog)
// ─────────────────────────────────────────────────────────────────────────────
class Subsystem1507Test {

    private static final CANBus BUS = new CANBus(CANPort.CAN_S0);
    private static final Scheduler scheduler = Scheduler.getDefault();
    /** Records everything logged, so the tests can check it. */
    private static final MockTelemetryBackend telemetry = new MockTelemetryBackend();

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
        TelemetryRegistry.registerBackend("", telemetry);
        CommandLog.start(scheduler);   // LoggedRobot does this on the robot
        Subsystem1507.setDefaultCanBus(BUS);
    }

    private static String lastString(String path) {
        var value = telemetry.getLastValue(path, MockTelemetryBackend.LogStringValue.class);
        return value == null ? null : value.value();
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

    @Test
    void everyMotorSignalIsLoggedEveryLoop() {
        new TestArm(45);
        runLoops(1);

        for (Motor1507.MotorSignal signal : Motor1507.MotorSignal.values()) {
            assertNotNull(telemetry.getLastAction("/TestArm45/Arm/" + signal.logName),
                signal.logName + " was not logged");
        }
        assertNotNull(telemetry.getLastAction("/TestArm45/Arm/Stalled"));
        assertNotNull(telemetry.getLastAction("/TestArm45/TotalSupplyCurrent"));
        assertNotNull(telemetry.getLastAction("/TestArm45/PeriodicMs"));
    }

    @Test
    void theCommandUsingASubsystemIsLogged() {
        TestArm subsystem = new TestArm(46);
        Command hold = subsystem.run(coroutine -> coroutine.park()).named("TestArm46.hold");

        scheduler.schedule(hold);
        runLoops(1);
        assertEquals("TestArm46.hold", lastString("/TestArm46/Command"));
        assertEquals("START TestArm46.hold", lastString("/Commands/Events"));

        scheduler.cancel(hold);
        runLoops(1);
        assertEquals("", lastString("/TestArm46/Command"));
        assertEquals("CANCELED TestArm46.hold", lastString("/Commands/Events"));
    }
}
