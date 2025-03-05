package frc.robot.subsystems.drive.weights;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.RobotConstants.AutoAlignConfig;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class FollowPath {
  protected Supplier<Pose2d> robotPose;
  protected Gyro gyro;

  protected Command pathCommand = null;
  public Pose2d[] pose2dArray;
  public Rotation2d endRotation;
  private PathConstraints pathConstraints;
  private DriveSubsystem driveSubsystem;
  private ChassisSpeeds lastSpeeds = new ChassisSpeeds();
  IdealStartingState idealStartingState = null;

  public FollowPath(
      Supplier<Pose2d> robotPose,
      Gyro gyro,
      Pose2d[] pose2dArray,
      PathConstraints pathConstraints,
      Rotation2d endRotation,
      DriveSubsystem driveSubsystem) {
    this.robotPose = robotPose;
    this.gyro = gyro;
    this.pose2dArray = pose2dArray;
    this.pathConstraints = pathConstraints;
    this.endRotation = endRotation;
    this.driveSubsystem = driveSubsystem;
    new Trigger(() -> !isNearSetpoint() && pathCommand != null)
        .onTrue(new InstantCommand(() -> restartPath()));

    new Trigger(() -> isAutoalignComplete() && pathCommand != null)
        .onTrue(new InstantCommand(() -> stopPath()));
  }

  public void restartPath() {
    stopPath();
    // idealStartingState =
    //     new IdealStartingState(
    //         ChassisSpeedHelper.magnitude(lastSpeeds), robotPose.get().getRotation());
    idealStartingState = null;
    startPath();
  }

  public void stopPath() {
    if (pathCommand != null) {
      pathCommand.cancel();
    }
    lastSpeeds = PathplannerWeight.getSpeedsPath();
    PathplannerWeight.setSpeeds(new ChassisSpeeds());
    pathCommand = null;
    idealStartingState = null;
  }

  public void startPath() {
    List<Waypoint> waypoints;
    ArrayList<Pose2d> waypointPos = new ArrayList<>();
    for (Pose2d waypointPose2d : pose2dArray) {
      Logger.recordOutput("waypointPose2d", waypointPose2d.getRotation().getDegrees());
      waypointPos.add(waypointPose2d);
    }
    if (waypointPos.size() <= 0) {
      stopPath();
      return;
    }
    if (robotPose.get().getTranslation().getDistance(waypointPos.get(0).getTranslation()) < 0.01) {
      stopPath();
      return;
    }
    Rotation2d angle =
        waypointPos.get(0).getTranslation().minus(robotPose.get().getTranslation()).getAngle();
    waypointPos.add(0, new Pose2d(robotPose.get().getTranslation(), angle));

    waypoints = PathPlannerPath.waypointsFromPoses(waypointPos);
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints, pathConstraints, idealStartingState, new GoalEndState(0.00, endRotation));
    path.preventFlipping = true;
    if (pathCommand == null) {
      pathCommand = AutoBuilder.followPath(path);
      pathCommand.schedule();
    }
  }

  public Trigger generateTrigger(BooleanSupplier condition) {
    return new Trigger(condition)
        .onTrue(new InstantCommand(() -> startPath()))
        .onFalse(new InstantCommand(() -> stopPath()));
  }

  public boolean isNearSetpoint() {
    return PathplannerWeight.setpoint.getTranslation().getDistance(robotPose.get().getTranslation())
        < AutoAlignConfig.maxDistanceFromTarget;
  }

  public Pose2d getFinalPosition() {
    return pose2dArray[pose2dArray.length - 1];
  }

  public boolean isEnabled() {
    return pathCommand != null;
  }

  public boolean isAutoalignComplete() {
    if (pose2dArray == null) {
      return false;
    }
    if (pose2dArray.length == 0) {
      return false;
    }
    Pose2d target = getFinalPosition();
    Pose2d robot = robotPose.get();
    boolean complete =
        (target.getTranslation().getDistance(robot.getTranslation()) < 0.2
            && Math.abs(target.getRotation().minus(robot.getRotation()).getDegrees()) < 1);
    ChassisSpeeds chassisSpeeds = driveSubsystem.getChassisSpeeds();
    complete &=
        Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond) < 0.005;
    return complete;
  }
}
