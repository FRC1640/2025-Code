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
        Logger.recordOutput(
            "PIDTrack/" + idName.get(i) + "/errorDerivativeTolerance",
            pidsTrack.get(i).getErrorDerivativeTolerance());
        Logger.recordOutput("PIDTrack/" + idName.get(i) + "/IZone", pidsTrack.get(i).getIZone());
        Logger.recordOutput(
            "PIDTrack/" + idName.get(i) + "/accumulatedError",
            pidsTrack.get(i).getAccumulatedError());
        Logger.recordOutput(
            "PIDTrack/" + idName.get(i) + "/continuosInputEnabled",
            pidsTrack.get(i).isContinuousInputEnabled());

        // Information of the Values
        Logger.recordOutput("PIDTrack/" + idName.get(i) + "/constants/kP", pidsTrack.get(i).getP());
        Logger.recordOutput("PIDTrack/" + idName.get(i) + "/constants/kI", pidsTrack.get(i).getI());
        Logger.recordOutput("PIDTrack/" + idName.get(i) + "/constants/kD", pidsTrack.get(i).getD());

        Logger.recordOutput("PIDTrack/" + idName.get(i) + "/output", calculate(pidsTrack.get(i)));
      }
    }
  }

  public static double calculate(PIDController controller) {
    double p = controller.getError();
    double i = controller.getAccumulatedError();
    double d = controller.getErrorDerivative();
    return controller.getP() * p + controller.getI() * i + controller.getD() * d;
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
        Logger.recordOutput("PIDTrack/" + idName.get(i) + "/IZone", pidsTrack.get(i).getIZone());
        Logger.recordOutput(
            "PIDTrack/" + idName.get(i) + "/constraints/maxAcceleration",
            pidsTrack.get(i).getConstraints().maxAcceleration);
        Logger.recordOutput(
            "PIDTrack/" + idName.get(i) + "/constraints/maxVelocity",
            pidsTrack.get(i).getConstraints().maxVelocity);
        Logger.recordOutput(
            "PIDTrack/" + idName.get(i) + "/goal/position", pidsTrack.get(i).getGoal().position);
        Logger.recordOutput(
            "PIDTrack/" + idName.get(i) + "/goal/velocity", pidsTrack.get(i).getGoal().velocity);
        Logger.recordOutput(
            "PIDTrack/" + idName.get(i) + "/accumulatedError",
            pidsTrack.get(i).getAccumulatedError());
        // Information of the Values
        Logger.recordOutput("PIDTrack/" + idName.get(i) + "/constants/kP", pidsTrack.get(i).getP());
        Logger.recordOutput("PIDTrack/" + idName.get(i) + "/constants/kI", pidsTrack.get(i).getI());
        Logger.recordOutput("PIDTrack/" + idName.get(i) + "/constants/kD", pidsTrack.get(i).getD());
      }
    }
  }
}
