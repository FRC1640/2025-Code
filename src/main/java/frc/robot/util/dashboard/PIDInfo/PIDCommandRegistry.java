package frc.robot.util.dashboard.PIDInfo;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.logging.PIDTracking.PIDTrack;
import java.util.HashMap;
import java.util.function.Function;

public class PIDCommandRegistry {

  public static HashMap<PIDController, Function<Double, Command>> attachedCommands =
      new HashMap<>();
  public static Command currentlyRunningCommand = null;

  public static void attachPIDCommand(String pidID, Function<Double, Command> command) {
    attachedCommands.put(PIDTrack.pidsTrack.get(pidID), command);
  }
}
