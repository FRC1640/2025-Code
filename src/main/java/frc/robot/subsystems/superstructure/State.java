package frc.robot.subsystems.superstructure;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class State {
  private List<StatePointer> pointers;

  public State(StatePointer... pointers) {
    this.pointers = Arrays.asList(pointers);
  }

  public State() {
    pointers = new ArrayList<>();
  }

  public void addPointer(StatePointer pointer) {
    pointers.add(pointer);
  }

  public void addPointer(Trigger trigger, State state) {
    pointers.add(new StatePointer(trigger, state));
  }
}
