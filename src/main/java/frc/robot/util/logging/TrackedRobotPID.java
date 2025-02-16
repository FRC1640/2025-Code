package frc.robot.util.logging;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class TrackedRobotPID {
  // Regular
  public class PIDTrack {
    public static HashMap<String, PIDController> pidsTrack = new HashMap<String, PIDController>();

    public static void logValuesID() {
      for (String idName : pidsTrack.keySet()) {
        Logger.recordOutput("PIDTrack/" + idName + "/error", pidsTrack.get(idName).getError());
        Logger.recordOutput(
            "PIDTrack/" + idName + "/setPoint", pidsTrack.get(idName).getSetpoint());
        Logger.recordOutput(
            "PIDTrack/" + idName + "/atSetPoint", pidsTrack.get(idName).atSetpoint());
        Logger.recordOutput(
            "PIDTrack/" + idName + "/errorDerivative", pidsTrack.get(idName).getErrorDerivative());
        Logger.recordOutput(
            "PIDTrack/" + idName + "/errorTolerance", pidsTrack.get(idName).getErrorTolerance());
        Logger.recordOutput("PIDTrack/" + idName + "/period", pidsTrack.get(idName).getPeriod());
        Logger.recordOutput(
            "PIDTrack/" + idName + "/errorDerivativeTolerance",
            pidsTrack.get(idName).getErrorDerivativeTolerance());
        Logger.recordOutput("PIDTrack/" + idName + "/IZone", pidsTrack.get(idName).getIZone());
        Logger.recordOutput(
            "PIDTrack/" + idName + "/accumulatedError",
            pidsTrack.get(idName).getAccumulatedError());
        Logger.recordOutput(
            "PIDTrack/" + idName + "/continuosInputEnabled",
            pidsTrack.get(idName).isContinuousInputEnabled());

        // Information of the Values
        Logger.recordOutput("PIDTrack/" + idName + "/constants/kP", pidsTrack.get(idName).getP());
        Logger.recordOutput("PIDTrack/" + idName + "/constants/kI", pidsTrack.get(idName).getI());
        Logger.recordOutput("PIDTrack/" + idName + "/constants/kD", pidsTrack.get(idName).getD());

        Logger.recordOutput("PIDTrack/" + idName + "/output", calculate(pidsTrack.get(idName)));

        Logger.recordOutput(
            "PIDTrack/" + idName + "/outputDerivative", calculateDerivative(pidsTrack.get(idName)));

        Logger.recordOutput(
            "PIDTrack/" + idName + "/outputIntegral", calculateIntegral(pidsTrack.get(idName)));

        Logger.recordOutput(
            "PIDTrack/" + idName + "/outputProportional",
            calculateProportional(pidsTrack.get(idName)));
      }
    }
  }

  public static double calculate(PIDController controller) {
    double p = controller.getError();
    double i = controller.getAccumulatedError();
    double d = controller.getErrorDerivative();
    return controller.getP() * p + controller.getI() * i + controller.getD() * d;
  }

  public static double calculateDerivative(PIDController controller) {
    double d = controller.getErrorDerivative();
    return controller.getD() * d;
  }

  public static double calculateIntegral(PIDController controller) {
    double i = controller.getAccumulatedError();
    return controller.getI() * i;
  }

  public static double calculateProportional(PIDController controller) {
    double p = controller.getError();
    return controller.getP() * p;
  }

  // Profiled
  public class ProfiledPIDTrack {
    public static HashMap<String, ProfiledPIDController> pidsTrack =
        new HashMap<String, ProfiledPIDController>();

    public static void logValuesID() {
      for (String idName : pidsTrack.keySet()) {
        Logger.recordOutput("PIDTrack/" + idName + "/profiledPID", true);
        Logger.recordOutput(
            "PIDTrack/" + idName + "/positionError", pidsTrack.get(idName).getPositionError());
        Logger.recordOutput(
            "PIDTrack/" + idName + "/velocityError", pidsTrack.get(idName).getVelocityError());
        Logger.recordOutput(
            "PIDTrack/" + idName + "/atSetpoint", pidsTrack.get(idName).atSetpoint());
        Logger.recordOutput("PIDTrack/" + idName + "/period", pidsTrack.get(idName).getPeriod());
        Logger.recordOutput(
            "PIDTrack/" + idName + "/velocityTolerance",
            pidsTrack.get(idName).getVelocityTolerance());
        Logger.recordOutput(
            "PIDTrack/" + idName + "/positionTolerance",
            pidsTrack.get(idName).getPositionTolerance());
        Logger.recordOutput("PIDTrack/" + idName + "/IZone", pidsTrack.get(idName).getIZone());
        Logger.recordOutput(
            "PIDTrack/" + idName + "/constraints/maxAcceleration",
            pidsTrack.get(idName).getConstraints().maxAcceleration);
        Logger.recordOutput(
            "PIDTrack/" + idName + "/constraints/maxVelocity",
            pidsTrack.get(idName).getConstraints().maxVelocity);
        Logger.recordOutput(
            "PIDTrack/" + idName + "/goal/position", pidsTrack.get(idName).getGoal().position);
        Logger.recordOutput(
            "PIDTrack/" + idName + "/goal/velocity", pidsTrack.get(idName).getGoal().velocity);
        Logger.recordOutput(
            "PIDTrack/" + idName + "/accumulatedError",
            pidsTrack.get(idName).getAccumulatedError());
        // Information of the Values
        Logger.recordOutput("PIDTrack/" + idName + "/constants/kP", pidsTrack.get(idName).getP());
        Logger.recordOutput("PIDTrack/" + idName + "/constants/kI", pidsTrack.get(idName).getI());
        Logger.recordOutput("PIDTrack/" + idName + "/constants/kD", pidsTrack.get(idName).getD());
      }
    }
  }
}
