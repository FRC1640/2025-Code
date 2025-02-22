package frc.robot.util.logging;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import java.util.ArrayList;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;

public class TrackedFeedForward {
  public class FeedForwardTrack {
    public static ArrayList<SimpleMotorFeedforward> feedTrack =
        new ArrayList<SimpleMotorFeedforward>();

    public static ArrayList<String> idName = new ArrayList<String>();

    public static void logVal() {
      for (int i = 0; i < feedTrack.size(); i++) {
        Logger.recordOutput(
            "SimpleFeedForward/" + idName.get(i) + "/period (Dt)",
            getDoubleValues(SimpleMotorFeedforward::getDt, i));
        Logger.recordOutput(
            "SimpleFeedForward/" + idName.get(i) + "/accelerationGain (Ka)",
            getDoubleValues(SimpleMotorFeedforward::getKa, i));
        Logger.recordOutput(
            "SimpleFeedForward/" + idName.get(i) + "/staticGain (Ks)",
            getDoubleValues(SimpleMotorFeedforward::getKs, i));
        Logger.recordOutput(
            "SimpleFeedForward/" + idName.get(i) + "/velocityGain (Kv)",
            getDoubleValues(SimpleMotorFeedforward::getKv, i));
      }
    }

    public static double getDoubleValues(
        Function<SimpleMotorFeedforward, Double> function, int id) {
      return function.apply(feedTrack.get(id));
    }

    public static boolean getBooleanValues(
        Function<SimpleMotorFeedforward, Boolean> function, int id) {
      return function.apply(feedTrack.get(id));
    }
  }

  public class ElevatorFeedForwardTrack {
    public static ArrayList<ElevatorFeedforward> elevatorFeedTrack =
        new ArrayList<ElevatorFeedforward>();

    public static ArrayList<String> idName = new ArrayList<String>();

    public static void logVal() {
      for (int i = 0; i < elevatorFeedTrack.size(); i++) {
        Logger.recordOutput(
            "ElevatorFeedForward/" + idName.get(i) + "/period (Dt)",
            getDoubleValues(ElevatorFeedforward::getDt, i));
        Logger.recordOutput(
            "ElevatorFeedForward/" + idName.get(i) + "/accelerationGain (Ka)",
            getDoubleValues(ElevatorFeedforward::getKa, i));
        Logger.recordOutput(
            "ElevatorFeedForward/" + idName.get(i) + "/gravityGain (Kg)",
            getDoubleValues(ElevatorFeedforward::getKg, i));
        Logger.recordOutput(
            "ElevatorFeedForward/" + idName.get(i) + "/staticGain (Ks)",
            getDoubleValues(ElevatorFeedforward::getKs, i));
        Logger.recordOutput(
            "ElevatorFeedForward/" + idName.get(i) + "/velocityGain (Kv)",
            getDoubleValues(ElevatorFeedforward::getKv, i));
      }
    }

    public static double getDoubleValues(Function<ElevatorFeedforward, Double> function, int id) {
      return function.apply(elevatorFeedTrack.get(id));
    }

    public static boolean getBooleanValues(
        Function<ElevatorFeedforward, Boolean> function, int id) {
      return function.apply(elevatorFeedTrack.get(id));
    }
  }
}
