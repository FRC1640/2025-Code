package frc.robot.subsystems.drive.weights;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.sensors.gyro.Gyro;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class FollowPath {
  protected Supplier<Pose2d> robotPose;
  protected Gyro gyro;

  protected Command pathCommand = null;
  public Pose2d[] pose2dArray;
  public Rotation2d endRotation;

  public FollowPath(
      Supplier<Pose2d> robotPose,
      Gyro gyro,
      Pose2d[] pose2dArray,
      double maxLinearVelocity,
      double maxLinearAcceleration,
      double maxAngularVelocity,
      double maxAngularAcceleration,
      Rotation2d endRotation) {
    this.robotPose = robotPose;
    this.gyro = gyro;
    this.pose2dArray = pose2dArray;
    this.endRotation = endRotation;
  }

  public void stopPath() {
    pathCommand.cancel();
    PathplannerWeight.setSpeeds(new ChassisSpeeds());
    pathCommand = null;
  }

  public void startPath() {
    List<Waypoint> waypoints;
    ArrayList<Pose2d> waypointPos = new ArrayList<Pose2d>();

    waypointPos.add(robotPose.get());
    for (Pose2d roboPose : pose2dArray) {
      waypointPos.add(roboPose);
    }

    waypoints = PathPlannerPath.waypointsFromPoses(waypointPos);
    PathConstraints pathConstraints =
        new PathConstraints(DriveConstants.maxSpeed, 3, DriveConstants.maxOmega, 4 * Math.PI);

    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            pathConstraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can
            // be null for on-the-fly paths.
            new GoalEndState(0.00, (endRotation)));
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
}
