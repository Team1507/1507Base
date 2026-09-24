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

    // TODO: Configure hardware
    // motor = new Motor1507(key("Motor"), Motor1507.Type.FX, RobotMap.MY_MOTOR, Constants.CAN_BUS, CONFIG);
  }

  // ---- Commands (Commands v3) ----
  //
  // Every command needs a name: .named("ReplaceMeSubsystem1507.action")
  // runRepeatedly(...) runs every loop until interrupted.
  // run(coroutine -> ...) runs top to bottom; call coroutine.yield() in every loop.

  /** Runs until interrupted (bind with whileTrue). */
  public Command exampleRunCommand() {
    return runRepeatedly(() -> {
          // TODO: set motor output every loop
        })
        .whenCanceled(() -> {
          // TODO: stop the motor
        })
        .named("ReplaceMeSubsystem1507.run");
  }

  /** Does one thing and finishes immediately (bind with onTrue). */
  public Command exampleOnceCommand() {
    return run(coroutine -> {
          // TODO: one-time action
        })
        .named("ReplaceMeSubsystem1507.once");
  }

  /** What this subsystem does when no command is using it. Set it as the default in Robot.java. */
  @Override
  public Command idle() {
    return run(coroutine -> {
          // TODO: stop the motor
          coroutine.park();
        })
        .withPriority(Command.LOWEST_PRIORITY)
        .named("ReplaceMeSubsystem1507.idle");
  }

  @Override
  public void periodic() {
    // Runs every loop, before commands.

    // TODO: Read each motor's latest values from CAN FIRST. Without this, every
    // motor getter (position, RPM, current) returns the same old value forever.
    // motor.refresh();

    // TODO: Log telemetry
    // log("someField", someValue);
  }

  @Override
  public void simulationPeriodic() {
    // Runs every loop in simulation only. Without this, motors don't move in sim.
    // motor.simulationPeriodic(0.02);
  }
}
