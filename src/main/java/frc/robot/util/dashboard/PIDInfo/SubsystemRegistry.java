package frc.robot.util.dashboard.PIDInfo;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.Subsystems;
import java.util.HashMap;
import java.util.function.Function;

public class SubsystemRegistry {

  public static HashMap<Subsystems, SubsystemBase> registry =
      new HashMap<Subsystems, SubsystemBase>();
  public static HashMap<PIDInfo, Function<SubsystemBase, Void>> mappedPIDFunction = new HashMap<>();

  public static void registerSubsystem(Subsystems subsystemKey, SubsystemBase subsystem) {
    registry.put(subsystemKey, subsystem);
  }

  public static void registerPIDFunction(PIDInfo pidInfo, Function<SubsystemBase, Void> function) {
    SubsystemBase subsystem = registry.get(pidInfo.connectedSubsystem);
    if (subsystem != null) {
      function.apply(subsystem);
    }
    mappedPIDFunction.put(pidInfo, function);
  }

  public static void executePIDFunction(PIDInfo pidInfo, SubsystemBase subsystem) {
    Function<SubsystemBase, Void> function = mappedPIDFunction.get(pidInfo);
    if (function != null) {
      function.apply(subsystem);
    }
  }
}
