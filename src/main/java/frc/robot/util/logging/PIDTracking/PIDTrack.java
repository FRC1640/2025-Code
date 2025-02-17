package frc.robot.util.logging.PIDTracking;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.dashboard.PIDInfo.PIDInfo;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

// Regular
public class PIDTrack {
  public static HashMap<PIDInfo, PIDController> pidsTrack = new HashMap<PIDInfo, PIDController>();

  public static void logValuesID() {
    for (PIDInfo idName : pidsTrack.keySet()) {
      Logger.recordOutput("PIDTrack/" + idName.name + "/error", pidsTrack.get(idName).getError());
      Logger.recordOutput(
          "PIDTrack/" + idName.name + "/setPoint", pidsTrack.get(idName).getSetpoint());
      Logger.recordOutput(
          "PIDTrack/" + idName.name + "/atSetPoint", pidsTrack.get(idName).atSetpoint());
      Logger.recordOutput(
          "PIDTrack/" + idName.name + "/errorDerivative",
          pidsTrack.get(idName).getErrorDerivative());
      Logger.recordOutput(
          "PIDTrack/" + idName.name + "/errorTolerance", pidsTrack.get(idName).getErrorTolerance());
      Logger.recordOutput("PIDTrack/" + idName.name + "/period", pidsTrack.get(idName).getPeriod());
      Logger.recordOutput(
          "PIDTrack/" + idName.name + "/errorDerivativeTolerance",
          pidsTrack.get(idName).getErrorDerivativeTolerance());
      Logger.recordOutput("PIDTrack/" + idName.name + "/IZone", pidsTrack.get(idName).getIZone());
      Logger.recordOutput(
          "PIDTrack/" + idName.name + "/accumulatedError",
          pidsTrack.get(idName).getAccumulatedError());
      Logger.recordOutput(
          "PIDTrack/" + idName.name + "/continuosInputEnabled",
          pidsTrack.get(idName).isContinuousInputEnabled());

      // Information of the Values
      Logger.recordOutput(
          "PIDTrack/" + idName.name + "/constants/kP", pidsTrack.get(idName).getP());
      Logger.recordOutput(
          "PIDTrack/" + idName.name + "/constants/kI", pidsTrack.get(idName).getI());
      Logger.recordOutput(
          "PIDTrack/" + idName.name + "/constants/kD", pidsTrack.get(idName).getD());

      Logger.recordOutput(
          "PIDTrack/" + idName.name + "/output", TrackedRobotPID.calculate(pidsTrack.get(idName)));

      Logger.recordOutput(
          "PIDTrack/" + idName.name + "/outputDerivative",
          TrackedRobotPID.calculateDerivative(pidsTrack.get(idName)));

      Logger.recordOutput(
          "PIDTrack/" + idName.name + "/outputIntegral",
          TrackedRobotPID.calculateIntegral(pidsTrack.get(idName)));

      Logger.recordOutput(
          "PIDTrack/" + idName.name + "/outputProportional",
          TrackedRobotPID.calculateProportional(pidsTrack.get(idName)));
    }
  }
}
