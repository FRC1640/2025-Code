package frc.robot.util.dashboard;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import frc.robot.util.periodic.PeriodicBase;
import java.util.HashMap;

public class PIDMap extends PeriodicBase {
  private static GenericEntry kP;
  private static GenericEntry kI;
  private static GenericEntry kD;
  private static GenericEntry setpt;
  private static PIDController pid;

  public static HashMap<String, PIDController> pidMap = new HashMap<>();

  @Override
  public void periodic() {
    pid.setP(kP.getDouble(0));
    pid.setI(kI.getDouble(0));
    pid.setD(kD.getDouble(0));
  }

  public static void setEntries(GenericEntry p, GenericEntry i, GenericEntry d, GenericEntry s) {
    kP = p;
    kI = i;
    kD = d;
    setpt = s;
  }

  public static void setPID(PIDController pidController) {
    pid = pidController;
  }
}
