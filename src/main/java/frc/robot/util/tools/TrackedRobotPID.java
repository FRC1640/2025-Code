package frc.robot.util.tools;

import edu.wpi.first.math.controller.PIDController;
import java.util.ArrayList;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;

public class TrackedRobotPID {
  // Regular
  public class PID {
    public static ArrayList<PIDController> pidsTrack = new ArrayList<PIDController>();

    public static ArrayList<String> idName = new ArrayList<String>();

    public static String[] getIDNames() {
      String[] idNameArray = new String[idName.size()];
      for (int i = 0; i < idName.size(); i++) {
        idNameArray[i] = idName.get(i);
      }
      return idNameArray;
    }

    public static boolean[] getAtSetPoint() {
      boolean[] atSetPoint = new boolean[pidsTrack.size()];
      for (int i = 0; i < pidsTrack.size(); i++) {
        atSetPoint[i] = pidsTrack.get(i).atSetpoint();
      }
      return atSetPoint;
    }

    public static double[] getValues(Function<PIDController, Double> function) {
      return pidsTrack.stream().mapToDouble(function::apply).toArray();
    }
  }

  public static void updateLogPID() {
    // Regular PIDs
    Logger.recordOutput("PIDTrack/PID/id", PID.getIDNames());

    Logger.recordOutput("PIDTrack/PID/info/error", PID.getValues(PIDController::getError));
    Logger.recordOutput("PIDTrack/PID/info/setPoint", PID.getValues(PIDController::getSetpoint));
    Logger.recordOutput("PIDTrack/PID/info/atSetPoint", PID.getAtSetPoint());
    Logger.recordOutput(
        "PIDTrack/PID/info/errorDerivative", PID.getValues(PIDController::getErrorDerivative));
    Logger.recordOutput(
        "PIDTrack/PID/info/errorTolerance", PID.getValues(PIDController::getErrorTolerance));
    Logger.recordOutput("PIDTrack/PID/info/period", PID.getValues(PIDController::getPeriod));
    // Information of the Values
    Logger.recordOutput("PIDTrack/PID/info/setValues/kP", PID.getValues(PIDController::getP));
    Logger.recordOutput("PIDTrack/PID/info/setValues/kI", PID.getValues(PIDController::getI));
    Logger.recordOutput("PIDTrack/PID/info/setValues/kD", PID.getValues(PIDController::getD));
  }
}
