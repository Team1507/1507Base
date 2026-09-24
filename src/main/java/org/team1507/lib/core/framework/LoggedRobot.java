//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.framework;

import org.wpilib.system.DataLogManager;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.command3.Scheduler;

import org.team1507.lib.core.logging.CommandLog;
import org.team1507.lib.core.logging.RobotHealthLog;

/**
 * Base robot class for Team 1507.
 *
 * <p>2027: extends {@link OpModeRobot} instead of TimedRobot. Autonomous and teleop
 * behavior live in OpMode classes (see AutoOpMode and robot/teleop/), which the
 * Driver Station lists in its drop-downs.
 *
 * <h2>Logging</h2>
 * Everything is recorded to one match log (a {@code .wpilog} file) that
 * AdvantageScope opens, and is also live on NetworkTables under
 * {@code /Telemetry}:
 * <ul>
 *   <li>Driver Station: joysticks, enabled/mode, match info ({@code DriverStation.startDataLog})</li>
 *   <li>Every subsystem's values and every motor's signals (Subsystem1507)</li>
 *   <li>Battery, brownout, loop timing, CAN buses, PDH (RobotHealthLog)</li>
 *   <li>Every command start/end (CommandLog)</li>
 * </ul>
 * On the robot, DataLogManager writes the log to a USB drive's {@code logs}
 * folder if one is plugged in, otherwise to {@code /home/systemcore/logs}. In
 * the simulator it goes to {@code logs/} in the project folder. The robot
 * program's console output is recorded in the same file.
 *
 * <h2>Commands</h2>
 * Runs the Commands v3 {@link Scheduler} every loop. Two v3 behaviors to know:
 * <ul>
 *   <li>Commands scheduled and triggers bound inside an OpMode are removed
 *       automatically when a different OpMode is selected.</li>
 *   <li>The v3 scheduler does not stop commands when the robot disables. This
 *       class cancels every running command on the enabled-to-disabled edge, so a
 *       half-finished auto never resumes on the next enable. (v2 did the same.)
 *       Commands started while disabled (e.g. zero heading) still run.</li>
 * </ul>
 */
public abstract class LoggedRobot extends OpModeRobot {

    private final Scheduler scheduler = Scheduler.getDefault();
    private final RobotHealthLog health;
    private boolean wasDisabled = true;

    protected LoggedRobot() {
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        CommandLog.start(scheduler);
        health = new RobotHealthLog();
    }

    /**
     * Logs the power distribution hub (PDH) on {@code port}: the current on
     * every channel, so a brownout can be traced to the mechanism that caused
     * it. Call once from Robot.java with the port the PDH is wired to.
     */
    protected void logPowerDistribution(CANPort port) {
        health.logPowerDistribution(port);
    }

    @Override
    public void robotPeriodic() {
        long loopStart = health.loopStart();

        // Cancel outside scheduler.run(): cancelAll() is not safe while a command is running.
        boolean disabled = isDisabled();
        if (disabled && !wasDisabled) {
            scheduler.cancelAll();
        }
        wasDisabled = disabled;

        scheduler.run();
        health.loopEnd(loopStart);
    }
}
