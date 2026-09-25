//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot;

import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;

// ─────────────────────────────────────────────────────────────────────────────
// TestRobot
//
// One simulated Robot shared by every test that needs the real subsystems
// (the drivetrain, autos). Building it takes a few seconds, and building two
// would put two drivetrains with the same CAN IDs on the simulated bus, so it
// is built once per test run.
//
// Also: time control. Tests call step() to run one 20 ms robot loop with
// simulated time paused and advanced exactly 20 ms, so anything timed (auto
// cutoffs, stuck detection) matches the simulated robot's motion.
// ─────────────────────────────────────────────────────────────────────────────
public final class TestRobot {

    private static Robot robot;

    private TestRobot() {}

    /** The shared simulated robot. */
    public static synchronized Robot get() {
        if (robot == null) {
            HAL.initialize();
            robot = new Robot();
        }
        return robot;
    }

    /** Runs one robot loop: 20 ms of simulated time, then the scheduler. */
    public static void step() {
        SimHooks.stepTiming(0.02);
        Scheduler.getDefault().run();
    }

    /** Runs {@code loops} robot loops. */
    public static void step(int loops) {
        for (int i = 0; i < loops; i++) {
            step();
        }
    }

    /** Pauses simulated time so step() controls it. Call before a timed test. */
    public static void pauseTime() {
        SimHooks.pauseTiming();
    }

    /** Lets simulated time run normally again. */
    public static void resumeTime() {
        SimHooks.resumeTiming();
    }

    /** Sets the alliance the Driver Station reports. */
    public static void setAlliance(boolean red) {
        DriverStationSim.setAllianceStationId(red ? AllianceStationID.RED_1 : AllianceStationID.BLUE_1);
        DriverStationSim.notifyNewData();
        DriverStationBackend.refreshData();   // the robot loop does this; tests must do it themselves
    }
}
