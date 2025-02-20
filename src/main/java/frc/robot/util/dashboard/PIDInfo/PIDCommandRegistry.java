package frc.robot.util.dashboard.PIDInfo;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.logging.PIDTracking.PIDTrack;
import frc.robot.util.logging.PIDTracking.ProfiledPIDTrack;
import java.util.HashMap;
import java.util.function.Function;

public class PIDCommandRegistry {

  public static HashMap<PIDController, Function<Double, Command>> attachedCommands =
      new HashMap<>();
  public static HashMap<ProfiledPIDController, Function<Double, Command>> profiledAttachedCommands =
      new HashMap<>();
  public static Command currentlyRunningCommand = null;

  public static void attachPIDCommand(String pidID, Function<Double, Command> command) {
    attachedCommands.put(PIDTrack.pidsTrack.get(pidID), command);
  }

  public static void attachProfiledPIDCommand(String pidID, Function<Double, Command> command) {
    profiledAttachedCommands.put(ProfiledPIDTrack.pidsTrack.get(pidID), command);
  }
}
