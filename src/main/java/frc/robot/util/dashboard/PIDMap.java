package frc.robot.util.dashboard;

import edu.wpi.first.math.controller.PIDController;
import java.util.HashMap;

public class PIDMap {
  public static enum PIDKey {
    DRIVE,
    STEER,
    LINEARDRIVE,
    ROTTORAD,
    GANTRY,
    LIFT,
    CORALOUTTAKE,
    CLIMBLIFT,
    CLIMBWINCH
  }

  public static HashMap<PIDKey, PIDController> pidMap;
}
