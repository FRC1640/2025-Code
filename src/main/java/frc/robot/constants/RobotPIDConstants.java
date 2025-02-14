package frc.robot.constants;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.util.control.FeedForwardConstants;
import frc.robot.util.spark.SparkPIDConstants;
import frc.robot.util.tools.logging.TrackedFeedForward.ElevatorFeedForwardTrack;
import frc.robot.util.tools.logging.TrackedFeedForward.FeedForwardTrack;
import frc.robot.util.tools.logging.TrackedRobotPID.PIDTrack;
import frc.robot.util.tools.logging.TrackedRobotPID.ProfiledPIDTrack;

public class RobotPIDConstants {
  public static final PIDController constructPID(PIDConstants constants) {
    PIDController j = new PIDController(constants.kP, constants.kI, constants.kD);
    PIDTrack.pidsTrack.add(j);
    PIDTrack.idName.add("PID" + (PIDTrack.pidsTrack.size()));
    return j;
  }

  public static final PIDController constructPID(PIDConstants constants, String pidTrackedName) {
    PIDController j = new PIDController(constants.kP, constants.kI, constants.kD);
    PIDTrack.pidsTrack.add(j);
    PIDTrack.idName.add(pidTrackedName);
    return j;
  }

  public static final SimpleMotorFeedforward constructFFSimpleMotor(
      FeedForwardConstants constants) {
    SimpleMotorFeedforward feedforward =
        new SimpleMotorFeedforward(constants.kS, constants.kV, constants.kA);
    FeedForwardTrack.feedTrack.add(feedforward);
    FeedForwardTrack.idName.add("SimpleMotorFeedForward" + FeedForwardTrack.feedTrack.size());
    return feedforward;
  }

  public static final SimpleMotorFeedforward constructFFSimpleMotor(
      FeedForwardConstants constants, String name) {
    SimpleMotorFeedforward feedforward =
        new SimpleMotorFeedforward(constants.kS, constants.kV, constants.kA);
    FeedForwardTrack.feedTrack.add(feedforward);
    FeedForwardTrack.idName.add(name);
    return feedforward;
  }

  public static final ProfiledPIDController constructProfiledPIDController(
      PIDConstants pidConstants, TrapezoidProfile.Constraints constraints) {
    ProfiledPIDController k =
        new ProfiledPIDController(
            pidConstants.kP, pidConstants.kI, pidConstants.kD, constraints, 0.02);
    ProfiledPIDTrack.pidsTrack.add(k);
    ProfiledPIDTrack.idName.add("PPID" + (ProfiledPIDTrack.pidsTrack.size()));

    return k;
  }

  public static final ProfiledPIDController costructProfiledPIDController(
      PIDConstants pidConstants, TrapezoidProfile.Constraints constraints, String name) {
    ProfiledPIDController k =
        new ProfiledPIDController(
            pidConstants.kP, pidConstants.kI, pidConstants.kD, constraints, 0.02);
    ProfiledPIDTrack.pidsTrack.add(k);
    ProfiledPIDTrack.idName.add("PPID" + (ProfiledPIDTrack.pidsTrack.size()));
    ProfiledPIDTrack.idName.add(name);

    return k;
  }

  public static final ElevatorFeedforward constructFFElevator(FeedForwardConstants constants) {
    ElevatorFeedforward k =
        new ElevatorFeedforward(constants.kS, constants.kG, constants.kV, constants.kA);
    ElevatorFeedForwardTrack.elevatorFeedTrack.add(k);
    ElevatorFeedForwardTrack.idName.add(
        "ElevatorFeedForward" + (ElevatorFeedForwardTrack.elevatorFeedTrack.size()));
    return k;
  }

  public static final MAXMotionConfig constructMaxMotionPos(PIDConstants constant) {
    MAXMotionConfig config = new MAXMotionConfig();

    config.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
    return config;
  }

  public static final PIDConstants drivePID = new PIDConstants(0.1546, 0.0, 0);
  public static final FeedForwardConstants driveFF =
      new FeedForwardConstants(0.21607, 2.6, 0.21035);

  public static final PIDConstants steerPID = new PIDConstants(0.725, 0.0, 0.005);

  public static final PIDConstants linearDrivePID = new PIDConstants(0.25, 0, 0);

  public static final PIDConstants linearDrivePIDProfiled = new PIDConstants(0.1, 0, 0);

  public static final PIDConstants rotateToAnglePIDRadians = new PIDConstants(0.5, 0.001, 0.0001);

  public static final PIDConstants gantryPID = new PIDConstants(50, 0, 0);
  public static final PIDConstants liftPID = new PIDConstants(0.001, 0, 0);
  public static final PIDConstants liftProfiledPIDConstants = new PIDConstants(27.25, 0.010569);
  public static final FeedForwardConstants liftFF = new FeedForwardConstants(0, 26.04, 0.0101, 0);

  public static final SparkPIDConstants pidConstantSpark =
      new SparkPIDConstants(0, 0, 0, -10, 10, ClosedLoopSlot.kSlot0).setVelocityFF(1);
  public static final FeedForwardConstants gantryFF =
      new FeedForwardConstants(0.19249, 20.698, 1.1805);
  public static final PIDConstants gantryVelocityPID = new PIDConstants(1.4935);
  public static final PIDConstants climberLiftPID = new PIDConstants(0.3546, 0, 0);
  public static final PIDConstants climberWinchPID = new PIDConstants(0.3546, 0, 0);

  public static final PIDConstants driveAntiTip = new PIDConstants(3, 0, 0);
}
