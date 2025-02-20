package frc.robot.util.logging;

import frc.robot.util.logging.PIDTracking.PIDTrack;
import frc.robot.util.logging.PIDTracking.ProfiledPIDTrack;
import frc.robot.util.logging.TrackedFeedForward.ElevatorFeedForwardTrack;
import frc.robot.util.logging.TrackedFeedForward.FeedForwardTrack;

public class LoggerManager {

  public static void updateLog() {
    updateLogPID();

    updateLogFF();
  }

  public static void updateLogPID() {
    // Regular PIDs
    PIDTrack.logValuesID();
    // Profiled PIDs
    ProfiledPIDTrack.logValuesID();
  }

  public static void updateLogFF() {
    // Elevator FeedForward
    ElevatorFeedForwardTrack.logVal();
    // Simple Feed Forward
    FeedForwardTrack.logVal();
  }
}
