package frc.robot.constants;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.util.control.FeedForwardConstants;

public class RobotPIDConstants {
  public static final PIDController constructPID(PIDConstants constants) {
    return new PIDController(constants.kP, constants.kI, constants.kD);
  }

  public static final SimpleMotorFeedforward constructFFSimpleMotor(
      FeedForwardConstants constants) {
    return new SimpleMotorFeedforward(constants.kS, constants.kV, constants.kA);
  }

  public static final ProfiledPIDController costructProfiledPIDController(
      PIDConstants pidConstants, TrapezoidProfile.Constraints constraints) {
    return new ProfiledPIDController(
        pidConstants.kP, pidConstants.kI, pidConstants.kD, constraints, 0.02);
  }

  public static final ElevatorFeedforward constructFFElevator(FeedForwardConstants constants) {
    return new ElevatorFeedforward(constants.kS, constants.kG, constants.kV, constants.kA);
  }

  public static final PIDConstants drivePID = new PIDConstants(0.1546, 0.0, 0);
  public static final FeedForwardConstants driveFF =
      new FeedForwardConstants(0.21607, 2.6, 0.21035);

  public static final PIDConstants steerPID = new PIDConstants(0.725, 0.0, 0.005);

  public static final PIDConstants linearDrivePID = new PIDConstants(0.8, 0, 0);

  public static final PIDConstants rotateToAnglePIDRadians = new PIDConstants(0.5, 0.001, 0.0001);

  public static final PIDConstants gantryPID = new PIDConstants(0, 0, 0);
  public static final PIDConstants liftPID = new PIDConstants(0.001, 0, 0);
  public static final PIDConstants liftProfiledPIDConstants = new PIDConstants(27.25, 0.010569);
  public static final FeedForwardConstants liftFF = new FeedForwardConstants(0, 26.04, 0.0101, 0);
}
