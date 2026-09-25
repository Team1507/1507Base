//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.framework;

import org.wpilib.command3.Command;
import org.wpilib.command3.Trigger;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.RobotState;
import org.wpilib.opmode.OpMode;

/**
 * Base class for autonomous routines (2027 OpMode framework, Commands v3).
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
 * <p>Lifecycle:
 * <ul>
 *   <li>WPILib constructs the class when the routine is <b>selected</b> on the
 *       Driver Station. The constructor binds an "enabled" trigger.</li>
 *   <li>When the robot is <b>enabled</b>, the trigger runs {@link #build()} and
 *       starts the command. Building at enable (not at selection) means the
 *       alliance color is known, so Red-alliance pose flipping is correct.</li>
 *   <li>When the robot <b>disables</b>, LoggedRobot cancels the command. When a
 *       different OpMode is selected, Commands v3 removes the trigger.</li>
 * </ul>
 *
 * <p>Warm-up: while the routine is selected and the robot is disabled, it is
 * built once in advance. The first build loads a lot of code (about 34 ms on a
 * laptop, more on SystemCore); doing it early means the robot doesn't sit still
 * at the start of auto. It also reports a routine with mistakes on the Driver
 * Station before the match, not at enable.
 */
public abstract class AutoOpMode implements OpMode {

    /** Builds the command this routine runs. Called each time the robot is enabled. */
    protected abstract Command build();

    private final String name = getClass().getSimpleName();
    private boolean warmedUp = false;

    protected AutoOpMode() {
        new Trigger(RobotState::isEnabled).onTrue(
            Command.noRequirements(coroutine -> coroutine.await(build())).named(name));
    }

    /** Runs every loop while this routine is selected and the robot is disabled. */
    @Override
    public void disabledPeriodic() {
        if (!warmedUp) {
            warmedUp = true;
            try {
                // Warm-up only: the auto builds again at enable. (Assigned because
                // WPILib's compiler check rejects a command that is made and dropped.)
                @SuppressWarnings("unused")
                Command warmUp = build();
            } catch (RuntimeException e) {
                DriverStationErrors.reportError("Auto '" + name + "' can't run: " + e.getMessage(), false);
            }
        }
    }
}
