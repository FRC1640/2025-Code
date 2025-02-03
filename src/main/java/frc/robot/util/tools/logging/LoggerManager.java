package frc.robot.util.tools.logging;

import frc.robot.util.tools.logging.TrackedFeedForward.ElevatorFeedForwardTrack;
import frc.robot.util.tools.logging.TrackedFeedForward.FeedForwardTrack;
import frc.robot.util.tools.logging.TrackedRobotPID.PIDTrack;
import frc.robot.util.tools.logging.TrackedRobotPID.ProfiledPIDTrack;

public class LoggerManager {

  public static void updateLogPID() {
    // Regular PIDs
    PIDTrack.logValuesID();
    // Profiled PIDs
    ProfiledPIDTrack.logValuesID();
    // Elevator FeedForward
    ElevatorFeedForwardTrack.logVal();
    // Simple Feed Forward
    FeedForwardTrack.logVal();
  }
}
