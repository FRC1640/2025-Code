package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.RobotConstants.LiftConstants.CoralPreset;
import frc.robot.util.controller.PresetBoard;

public class Superstructure {
  public static final State SAFE = new State();
  public static final State L4_RIGHT_PLACE = new State();

  /* state references */
  private static CommandXboxController driveController;
  private static CommandXboxController operatorController;
  private static PresetBoard presetBoard;

  private static CoralPreset coralPreset;

  private static State currentState;
  private static State requestedState;

  public static void requestState(State newState) {
    Superstructure.requestedState = newState;
  }

  static {
    Superstructure.SAFE.addPointer(operatorController.a().and(() -> coralPreset == CoralPreset.RightL4), L4_RIGHT_PLACE);
  }
}
