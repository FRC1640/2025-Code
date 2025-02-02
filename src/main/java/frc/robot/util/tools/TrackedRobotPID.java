package frc.robot.util.tools;

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
  }

  public static void updateLogPID() {
    // Regular PIDs
    Logger.recordOutput("PIDTrack/PID/id", PIDTrack.getIDNames());

    Logger.recordOutput("PIDTrack/PID/info/error", PIDTrack.getValues(PIDController::getError));
    Logger.recordOutput(
        "PIDTrack/PID/info/setPoint", PIDTrack.getValues(PIDController::getSetpoint));
    Logger.recordOutput("PIDTrack/PID/info/atSetPoint", PIDTrack.getAtSetPoint());
    Logger.recordOutput(
        "PIDTrack/PID/info/errorDerivative", PIDTrack.getValues(PIDController::getErrorDerivative));
    Logger.recordOutput(
        "PIDTrack/PID/info/errorTolerance", PIDTrack.getValues(PIDController::getErrorTolerance));
    Logger.recordOutput("PIDTrack/PID/info/period", PIDTrack.getValues(PIDController::getPeriod));
    // Information of the Values
    Logger.recordOutput("PIDTrack/PID/info/setValues/kP", PIDTrack.getValues(PIDController::getP));
    Logger.recordOutput("PIDTrack/PID/info/setValues/kI", PIDTrack.getValues(PIDController::getI));
    Logger.recordOutput("PIDTrack/PID/info/setValues/kD", PIDTrack.getValues(PIDController::getD));

    // Profiled PIDs
    Logger.recordOutput("PIDTrack/ProfiledPID/id", ProfiledPIDTrack.getIDNames());
    Logger.recordOutput(
        "PIDTrack/ProfiledPID/info/positionError",
        ProfiledPIDTrack.getValues(ProfiledPIDController::getPositionError));
    Logger.recordOutput(
        "PIDTrack/ProfiledPID/info/getVelocityError",
        ProfiledPIDTrack.getValues(ProfiledPIDController::getVelocityError));
    Logger.recordOutput(
        "PIDTrack/ProfiledPID/info/accumulatedError",
        ProfiledPIDTrack.getValues(ProfiledPIDController::getAccumulatedError));
    Logger.recordOutput(
        "PIDTrack/ProfiledPID/info/velocityTolerance",
        ProfiledPIDTrack.getValues(ProfiledPIDController::getVelocityTolerance));
    Logger.recordOutput(
        "PIDTrack/ProfiledPID/info/positionTolerance",
        ProfiledPIDTrack.getValues(ProfiledPIDController::getPositionTolerance));
    Logger.recordOutput(
        "PIDTrack/ProfiledPID/info/period",
        ProfiledPIDTrack.getValues(ProfiledPIDController::getPeriod));
    // Information of the Values
    Logger.recordOutput(
        "PIDTrack/ProfiledPID/info/setValues/kP",
        ProfiledPIDTrack.getValues(ProfiledPIDController::getP));
    Logger.recordOutput(
        "PIDTrack/ProfiledPID/info/setValues/kI",
        ProfiledPIDTrack.getValues(ProfiledPIDController::getI));
    Logger.recordOutput(
        "PIDTrack/ProfiledPID/info/setValues/kD",
        ProfiledPIDTrack.getValues(ProfiledPIDController::getD));
  }
}
