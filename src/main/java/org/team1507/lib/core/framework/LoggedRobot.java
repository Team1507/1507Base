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
import org.wpilib.command3.Scheduler;
import org.team1507.lib.core.logging.Telemetry;

/**
 * Base robot class for Team 1507.
 *
 * <p>2027: extends {@link OpModeRobot} instead of TimedRobot. Autonomous and teleop
 * behavior live in OpMode classes (see AutoOpMode and robot/teleop/), which the
 * Driver Station lists in its drop-downs.
 *
 * <p>Runs the Commands v3 {@link Scheduler} every loop. Two v3 behaviors to know:
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
    private boolean wasDisabled = true;

    protected LoggedRobot() {
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
    }

    @Override
    public void robotPeriodic() {
        // Cancel outside scheduler.run(): cancelAll() is not safe while a command is running.
        boolean disabled = isDisabled();
        if (disabled && !wasDisabled) {
            scheduler.cancelAll();
        }
        wasDisabled = disabled;

        scheduler.run();
        Telemetry.update();
    }
}
