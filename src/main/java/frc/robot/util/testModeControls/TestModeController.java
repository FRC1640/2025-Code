package frc.robot.util.testModeControls;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import java.util.function.BooleanSupplier;

public class TestModeController {
  static boolean constructTriggers = true;
  public static BooleanSupplier reconstructTrigger;

  public static void reconstructTriggersKeybind(RobotContainer robotContainer) {
    if (reconstructTrigger.getAsBoolean()) {
      if (constructTriggers) {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().getActiveButtonLoop().clear();
        constructTriggers = false;
      } else {
        robotContainer.reconstructKeybinds();
        constructTriggers = true;
      }
    }
  }
}
