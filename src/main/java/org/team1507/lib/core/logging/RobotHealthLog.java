//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.logging;

import com.ctre.phoenix6.CANBus;

import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.hal.can.CANStatus;
import org.wpilib.hardware.power.PowerDistribution;
import org.wpilib.system.RobotController;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;

import org.team1507.lib.core.framework.Subsystem1507;

/**
 * Logs the whole robot's health every loop: the data needed to explain a
 * brownout or a loop overrun after a match. LoggedRobot runs it; nothing else
 * needs to.
 *
 * <p>Logged under {@code Robot/}:
 * <table>
 *   <tr><td>BatteryVoltage, BrownedOut</td><td>every loop</td></tr>
 *   <tr><td>MotorSupplyCurrent — every Subsystem1507's motors added up</td><td>every loop</td></tr>
 *   <tr><td>LoopPeriodMs — time since the last loop started (20 = on time; more = overrun)</td><td>every loop</td></tr>
 *   <tr><td>RobotPeriodicMs — time spent in the scheduler (subsystems + commands)</td><td>every loop</td></tr>
 *   <tr><td>CAN/CAN_S0..S4/Utilization (0 to 1; 1 = full), BusOff, TxFull, ReceiveErrors, TransmitErrors</td><td>10 Hz</td></tr>
 *   <tr><td>CAN/&lt;CANivore name&gt;/... — the same, when the robot's default CAN bus
 *       (Subsystem1507.setDefaultCanBus) is a CANivore</td><td>10 Hz</td></tr>
 *   <tr><td>CPUTempC, CommsDisableCount</td><td>1 Hz</td></tr>
 * </table>
 * And under {@code PDH/} (only if Robot.java calls {@code logPowerDistribution}):
 * per-channel currents, total current and voltage, every loop.
 *
 * <p>Each subsystem also logs its own {@code TotalSupplyCurrent} and every
 * motor's currents and voltages (see Subsystem1507), and CommandLog logs what
 * each subsystem was doing.
 */
public final class RobotHealthLog {

    /** SystemCore's five CAN ports. (CANPort also lists CAN_D*, which aren't physical ports.) */
    private static final CANPort[] CAN_PORTS = {
        CANPort.CAN_S0, CANPort.CAN_S1, CANPort.CAN_S2, CANPort.CAN_S3, CANPort.CAN_S4
    };
    private static final int CAN_EVERY_N_LOOPS = 5;    // 10 Hz
    private static final int SLOW_EVERY_N_LOOPS = 50;  // 1 Hz

    private final TelemetryTable table = Telemetry.getTable("Robot");
    private final TelemetryTable[] canTables = new TelemetryTable[CAN_PORTS.length];

    private PowerDistribution pdh = null;

    /** The default CAN bus if it is a CANivore, found on the first CAN log; null otherwise. */
    private CANBus canivore = null;
    private TelemetryTable canivoreTable = null;
    private boolean canivoreChecked = false;

    private long lastLoopStartNanos = 0;
    private int loopCount = 0;

    public RobotHealthLog() {
        for (int i = 0; i < CAN_PORTS.length; i++) {
            canTables[i] = Telemetry.getTable("Robot/CAN/" + CAN_PORTS[i].name());
        }
    }

    /**
     * Also logs the power distribution hub on {@code port}: the current on every
     * channel, which shows exactly which mechanism pulled the battery down.
     * Detects the PDH at its default CAN ID (1 for REV).
     */
    public void logPowerDistribution(CANPort port) {
        pdh = new PowerDistribution(port);
    }

    /** Call first thing in every loop. Returns the start time for {@link #loopEnd}. */
    public long loopStart() {
        long now = System.nanoTime();
        if (lastLoopStartNanos != 0) {
            table.log("LoopPeriodMs", (now - lastLoopStartNanos) / 1e6);
        }
        lastLoopStartNanos = now;
        return now;
    }

    /** Call last thing in every loop, after the scheduler has run. */
    public void loopEnd(long loopStartNanos) {
        table.log("BatteryVoltage", RobotController.getBatteryVoltage());
        table.log("BrownedOut", RobotController.isBrownedOut());

        double motorCurrent = 0.0;
        for (Subsystem1507 subsystem : Subsystem1507.all()) {
            motorCurrent += subsystem.getSupplyCurrent();
        }
        table.log("MotorSupplyCurrent", motorCurrent);

        if (pdh != null) {
            Telemetry.log("PDH", pdh);
        }

        if (loopCount % CAN_EVERY_N_LOOPS == 0) {
            logCan();
        }
        if (loopCount % SLOW_EVERY_N_LOOPS == 0) {
            table.log("CPUTempC", RobotController.getCPUTemp());
            table.log("CommsDisableCount", RobotController.getCommsDisableCount());
        }
        loopCount++;

        // Last, so it includes the logging above.
        table.log("RobotPeriodicMs", (System.nanoTime() - loopStartNanos) / 1e6);
    }

    private void logCan() {
        logCanivore();
        for (int i = 0; i < CAN_PORTS.length; i++) {
            CANStatus status = RobotController.getCANStatus(CAN_PORTS[i]);
            TelemetryTable t = canTables[i];
            t.log("Utilization", status.percentBusUtilization);
            t.log("BusOff", status.busOffCount);
            t.log("TxFull", status.txFullCount);
            t.log("ReceiveErrors", status.receiveErrorCount);
            t.log("TransmitErrors", status.transmitErrorCount);
        }
    }

    /**
     * SystemCore's own ports are logged above. A CANivore (a USB CAN adapter,
     * e.g. the 2026 robot's "CAN-EATER") is only visible to CTRE's library, so
     * it is logged here. SystemCore port buses are named "can_s0".."can_s4".
     */
    private void logCanivore() {
        if (!canivoreChecked) {
            canivoreChecked = true;
            CANBus bus = Subsystem1507.getDefaultCanBus();
            if (bus != null && !bus.getName().toLowerCase().startsWith("can_s")) {
                canivore = bus;
                canivoreTable = Telemetry.getTable("Robot/CAN/" + bus.getName());
            }
        }
        if (canivore == null) {
            return;
        }
        CANBus.CANBusStatus status = canivore.getStatus();
        if (!status.Status.isOK()) {
            return;
        }
        canivoreTable.log("Utilization", status.BusUtilization);
        canivoreTable.log("BusOff", status.BusOffCount);
        canivoreTable.log("TxFull", status.TxFullCount);
        canivoreTable.log("ReceiveErrors", status.REC);
        canivoreTable.log("TransmitErrors", status.TEC);
    }
}
