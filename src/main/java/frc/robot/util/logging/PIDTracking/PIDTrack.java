package frc.robot.util.logging.PIDTracking;

import edu.wpi.first.math.controller.PIDController;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

// Regular
public class PIDTrack {
  public static HashMap<String, PIDController> pidsTrack = new HashMap<String, PIDController>();

  public static void logValuesID() {
    for (String idName : pidsTrack.keySet()) {
      Logger.recordOutput("PIDTrack/" + idName + "/error", pidsTrack.get(idName).getError());
      Logger.recordOutput("PIDTrack/" + idName + "/setPoint", pidsTrack.get(idName).getSetpoint());
      Logger.recordOutput("PIDTrack/" + idName + "/atSetPoint", pidsTrack.get(idName).atSetpoint());
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
          "PIDTrack/" + idName + "/accumulatedError", pidsTrack.get(idName).getAccumulatedError());
      Logger.recordOutput(
          "PIDTrack/" + idName + "/continuosInputEnabled",
          pidsTrack.get(idName).isContinuousInputEnabled());

      // Information of the Values
      Logger.recordOutput("PIDTrack/" + idName + "/constants/kP", pidsTrack.get(idName).getP());
      Logger.recordOutput("PIDTrack/" + idName + "/constants/kI", pidsTrack.get(idName).getI());
      Logger.recordOutput("PIDTrack/" + idName + "/constants/kD", pidsTrack.get(idName).getD());

      Logger.recordOutput(
          "PIDTrack/" + idName + "/output", TrackedRobotPID.calculate(pidsTrack.get(idName)));

      Logger.recordOutput(
          "PIDTrack/" + idName + "/outputDerivative",
          TrackedRobotPID.calculateDerivative(pidsTrack.get(idName)));

      Logger.recordOutput(
          "PIDTrack/" + idName + "/outputIntegral",
          TrackedRobotPID.calculateIntegral(pidsTrack.get(idName)));

      Logger.recordOutput(
          "PIDTrack/" + idName + "/outputProportional",
          TrackedRobotPID.calculateProportional(pidsTrack.get(idName)));
    }
  }
}
