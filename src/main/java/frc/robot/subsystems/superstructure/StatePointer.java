package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class StatePointer {
  private Trigger trigger;
  private State state;

  public StatePointer(Trigger trigger, State state) {
    this.trigger = trigger;
    this.state = state;
    this.trigger.onTrue(new InstantCommand(() -> Superstructure.requestState(state)));
  }
}
