package frc.robot.util.tools;

import edu.wpi.first.math.controller.PIDController;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class TrackedRobotPID {
  // Get the Position, Error Percent
  public class PID {
    public static ArrayList<PIDController> pidsTrack = new ArrayList<PIDController>();

    public static double[] getCurrentSetPositions() {
      double[] positionArray = new double[pidsTrack.size()];
      for (int i = 0; i < pidsTrack.size(); i++) {
        positionArray[i] = pidsTrack.get(i).getSetpoint();
      }
      return positionArray;
    }

    public static double[] getErrorRate() {
      double[] errorRateArray = new double[pidsTrack.size()];
      for (int i = 0; i < pidsTrack.size(); i++) {
        errorRateArray[i] = pidsTrack.get(i).getError();
      }
      return errorRateArray;
    }
  }

  public static void updateLogPID() {

    Logger.recordOutput("PIDTrack/position", PID.getCurrentSetPositions());
    Logger.recordOutput("PIDTrack/errorRate", PID.getErrorRate());
  }
}
