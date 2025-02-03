package frc.robot.util.tools.logging;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import java.util.ArrayList;

public class DebugPIDTuner {
  public static ArrayList<PIDConstants> pidConstants = new ArrayList<PIDConstants>();
  public static ArrayList<PIDController> pidControllers = new ArrayList<PIDController>();
}
