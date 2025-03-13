package frc.robot.util.logging.PIDTracking;

import edu.wpi.first.math.controller.ProfiledPIDController;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

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
      Logger.recordOutput("PIDTrack/" + idName + "/atSetpoint", pidsTrack.get(idName).atSetpoint());
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
          "PIDTrack/" + idName + "/accumulatedError", pidsTrack.get(idName).getAccumulatedError());
      // Information of the Values
      Logger.recordOutput("PIDTrack/" + idName + "/constants/kP", pidsTrack.get(idName).getP());
      Logger.recordOutput("PIDTrack/" + idName + "/constants/kI", pidsTrack.get(idName).getI());
      Logger.recordOutput("PIDTrack/" + idName + "/constants/kD", pidsTrack.get(idName).getD());
      Logger.recordOutput(
          "PIDTrack/" + idName + "/constants/maxAcceleration",
          pidsTrack.get(idName).getConstraints().maxAcceleration);
      Logger.recordOutput(
          "PIDTrack/" + idName + "/constants/maxVelocity",
          pidsTrack.get(idName).getConstraints().maxVelocity);
      Logger.recordOutput(
          "PIDTrack/" + idName + "/SetpointPos", pidsTrack.get(idName).getSetpoint().position);
    }
  }
}
