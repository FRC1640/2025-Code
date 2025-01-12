package frc.robot.constants;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.util.control.FeedForwardConstants;

public class RobotPIDConstants {
  public static final PIDController constructPID(PIDConstants constants) {
    return new PIDController(constants.kP, constants.kI, constants.kD);
  }

  public static final SimpleMotorFeedforward constructFF(FeedForwardConstants constants) {
    return new SimpleMotorFeedforward(constants.kS, constants.kV, constants.kA);
  }

  public static final PIDConstants drivePID = new PIDConstants(0.1546, 0.0, 0);
  public static final FeedForwardConstants driveFF =
      new FeedForwardConstants(0.21607, 2.6, 0.21035);

  public static final PIDConstants steerPID = new PIDConstants(0.725, 0.0, 0.005);

  public static final PIDConstants linearDrivePID = new PIDConstants(0.8, 0, 0);

  public static final PIDConstants rotateToAnglePIDRadians = new PIDConstants(0.5, 0.001, 0.0001);
}
