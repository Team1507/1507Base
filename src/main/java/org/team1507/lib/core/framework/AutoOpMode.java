//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.framework;

import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.opmode.PeriodicOpMode;

/**
 * Base class for autonomous routines (2027 OpMode framework).
 *
 * <p>Every auto routine is its own class that extends {@code AutoOpMode},
 * carries an {@code @Autonomous} annotation, and builds one command. WPILib finds
 * annotated classes automatically and lists them in the Driver Station's
 * autonomous drop-down, so there is no chooser to register them in.
 *
 * <pre>
 *   &#64;Autonomous(name = "Drive Forward")
 *   public final class DriveForwardAuto extends AutoOpMode {
 *       &#64;Override
 *       protected Command build() {
 *           return new AutoSequence()...build();
 *       }
 *   }
 * </pre>
 *
 * <p>Lifecycle: WPILib constructs the class when the routine is selected,
 * {@link #start()} runs once when the robot is enabled (schedules the command),
 * and {@link #end()} runs when the robot is disabled (cancels it).
 */
public abstract class AutoOpMode extends PeriodicOpMode {

    private Command command;

    /** Builds the command this routine runs. Called once, when auto starts. */
    protected abstract Command build();

    @Override
    public void start() {
        command = build();
        CommandScheduler.getInstance().schedule(command);
    }

    @Override
    public void end() {
        if (command != null) {
            command.cancel();
        }
    }
}
