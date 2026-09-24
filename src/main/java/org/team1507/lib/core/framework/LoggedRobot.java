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
import org.wpilib.framework.TimedRobot;
import org.wpilib.command2.CommandScheduler;
import org.team1507.lib.core.logging.Telemetry;

public abstract class LoggedRobot extends TimedRobot {

    protected LoggedRobot() {
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        Telemetry.update();
    }
}