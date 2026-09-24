//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.wpilib.commands.subsystem1507;

import org.wpilib.command3.Command;
import org.team1507.lib.core.framework.Subsystem1507;

public class ReplaceMeSubsystem1507 extends Subsystem1507 {

  // TODO: Declare hardware (motors, sensors, etc.) as private fields
  // private final Motor1507 motor;

  /** Creates a new ReplaceMeSubsystem1507. */
  public ReplaceMeSubsystem1507() {
    super("ReplaceMeSubsystem1507");

    // TODO: Create motors with motor(...). Subsystem1507 then refreshes them,
    // simulates them, and logs their total current every loop automatically.
    // motor = motor("Motor", Motor1507.Type.FX, RobotMap.MY_MOTOR, kMySubsystem.CONFIG);
  }

  // ---- Commands (Commands v3) ----
  //
  // Every command needs a name: .named("ReplaceMeSubsystem1507.action")
  // runRepeatedly(...) runs every loop until interrupted.
  // run(coroutine -> ...) runs top to bottom; call coroutine.yield() in every loop.

  /** Runs until interrupted (bind with whileTrue). */
  public Command exampleRunCommand() {
    return runRepeatedly(() -> {
          // TODO: set motor output every loop, e.g. motor.setRPM(3000);
        })
        .whenCanceled(() -> {
          // TODO: stop the motor, e.g. motor.stop();
        })
        .named("ReplaceMeSubsystem1507.run");
  }

  /** Moves to a target and finishes when it gets there (bind with onTrue). */
  public Command exampleMoveCommand() {
    return run(coroutine -> {
          // TODO: e.g. motor.setPosition(90);   // degrees
          // coroutine.waitUntil(motor::isAtTarget);
        })
        .named("ReplaceMeSubsystem1507.move");
  }

  /** What this subsystem does when no command is using it. Set it as the default in Robot.java. */
  @Override
  public Command idle() {
    return run(coroutine -> {
          // TODO: stop the motor, e.g. motor.stop();
          coroutine.park();
        })
        .withPriority(Command.LOWEST_PRIORITY)
        .named("ReplaceMeSubsystem1507.idle");
  }

  @Override
  public void periodic() {
    // Runs every loop, before commands. The motors were already refreshed.
    // TODO: Log telemetry, e.g. log("PositionDeg", motor.getPosition());
    // TODO: Flag problems, e.g. warnIf(motor.isStalled(), "Motor stalled");
  }
}
