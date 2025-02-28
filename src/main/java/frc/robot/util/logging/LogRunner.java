package frc.robot.util.logging;

import frc.robot.util.periodic.PeriodicBase;
import java.util.ArrayList;

public class LogRunner extends PeriodicBase {
  private static ArrayList<Loggable> logs = new ArrayList<>();

  public static void addLog(Loggable log) {
    logs.add(log);
  }

  @Override
  public void periodic() {
    for (Loggable logged : logs) {
      logged.log();
    }
  }
}
