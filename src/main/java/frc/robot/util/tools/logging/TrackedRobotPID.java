package frc.robot.util.tools.logging;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import java.util.ArrayList;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;

public class TrackedRobotPID {
  // Regular
  public class PIDTrack {
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

    public static void logValuesID() {
      for (int i = 0; i < pidsTrack.size(); i++) {
        Logger.recordOutput("PIDTrack/" + idName.get(i) + "/error", pidsTrack.get(i).getError());
        Logger.recordOutput(
            "PIDTrack/" + idName.get(i) + "/setPoint", pidsTrack.get(i).getSetpoint());
        Logger.recordOutput(
            "PIDTrack/" + idName.get(i) + "/atSetPoint", pidsTrack.get(i).atSetpoint());
        Logger.recordOutput(
            "PIDTrack/" + idName.get(i) + "/errorDerivative",
            pidsTrack.get(i).getErrorDerivative());
        Logger.recordOutput(
            "PIDTrack/" + idName.get(i) + "/errorTolerance", pidsTrack.get(i).getErrorTolerance());
        Logger.recordOutput("PIDTrack/" + idName.get(i) + "/period", pidsTrack.get(i).getPeriod());
        // Information of the Values
        Logger.recordOutput("PIDTrack/" + idName.get(i) + "/constants/kP", pidsTrack.get(i).getP());
        Logger.recordOutput("PIDTrack/" + idName.get(i) + "/constants/kI", pidsTrack.get(i).getI());
        Logger.recordOutput("PIDTrack/" + idName.get(i) + "/constants/kD", pidsTrack.get(i).getD());
      }
    }
  }

  // Profiled
  public class ProfiledPIDTrack {
    public static ArrayList<ProfiledPIDController> pidsTrack =
        new ArrayList<ProfiledPIDController>();

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

    public static double[] getValues(Function<ProfiledPIDController, Double> function) {
      return pidsTrack.stream().mapToDouble(function::apply).toArray();
    }

    public static void logValuesID() {
      for (int i = 0; i < pidsTrack.size(); i++) {
        Logger.recordOutput("PIDTrack/" + idName.get(i) + "/profiledPID", true);
        Logger.recordOutput(
            "PIDTrack/" + idName.get(i) + "/positionError", pidsTrack.get(i).getPositionError());
        Logger.recordOutput(
            "PIDTrack/" + idName.get(i) + "/velocityError", pidsTrack.get(i).getVelocityError());
        Logger.recordOutput(
            "PIDTrack/" + idName.get(i) + "/atSetpoint", pidsTrack.get(i).atSetpoint());
        Logger.recordOutput("PIDTrack/" + idName.get(i) + "/period", pidsTrack.get(i).getPeriod());
        Logger.recordOutput(
            "PIDTrack/" + idName.get(i) + "/velocityTolerance",
            pidsTrack.get(i).getVelocityTolerance());
        Logger.recordOutput(
            "PIDTrack/" + idName.get(i) + "/positionTolerance",
            pidsTrack.get(i).getPositionTolerance());
        // Information of the Values
        Logger.recordOutput("PIDTrack/" + idName.get(i) + "/constants/kP", pidsTrack.get(i).getP());
        Logger.recordOutput("PIDTrack/" + idName.get(i) + "/constants/kI", pidsTrack.get(i).getI());
        Logger.recordOutput("PIDTrack/" + idName.get(i) + "/constants/kD", pidsTrack.get(i).getD());
      }
    }
  }
}
