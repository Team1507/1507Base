//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package edu.wpi.first.wpilibj.commands.subsystem1507;

import edu.wpi.first.wpilibj2.command.Command;
import org.team1507.lib.core.framework.Subsystem1507;

public class ReplaceMeSubsystem1507 extends Subsystem1507 {

  // TODO: Declare hardware (motors, sensors, etc.) as private fields
  // private final Motor1507 m_motor;
  // private final BaseStatusSignal[] m_signals;

  /** Creates a new ReplaceMeSubsystem1507. */
  public ReplaceMeSubsystem1507() {
    super("ReplaceMeSubsystem1507");

    // TODO: Configure hardware
    // m_motor = new Motor1507(...);
  }

  // TODO: Add control methods
  // private void setOutput(double output) { ... }

  // ---- Command Factories ----

  public Command exampleCommand() {
    return runOnce(
        () -> {
          // TODO: Add command logic
        });
  }

  @Override
  public void periodic() {
    // TODO: Refresh hardware signals
    // BaseStatusSignal.refreshAll(m_signals);

    // TODO: Log telemetry
    // log("someField", someValue);
  }
}
