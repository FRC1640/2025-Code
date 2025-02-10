package frc.robot.subsystems.drive.weights;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.tools.DistanceManager;
import java.util.function.Function;
import java.util.function.Supplier;

public class FollowPathNearest extends FollowPath {
  Pose2d[] positions;
  Function<Pose2d, Pose2d> poseFunction;

  public FollowPathNearest(
      Supplier<Pose2d> robotPose,
      Gyro gyro,
      Pose2d[] positions,
      PathConstraints pathConstraints,
      Function<Pose2d, Pose2d> poseFunction,
      DriveSubsystem driveSubsystem) {
    super(robotPose, gyro, null, pathConstraints, null, driveSubsystem);
    this.positions = positions;
    pose2dArray = new Pose2d[] {findNearest(this.positions)};
    endRotation = findNearest(this.positions).getRotation();
    this.poseFunction = poseFunction;
  }

  public void setPoseFunction(Function<Pose2d, Pose2d> poseFunction) {
    this.poseFunction = poseFunction;
  }

  private Pose2d findNearest(Pose2d[] pos) {
    if (poseFunction != null) {
      return DistanceManager.getNearestPosition(robotPose.get(), pos, poseFunction);
    }
    return DistanceManager.getNearestPosition(robotPose.get(), pos);
  }

  @Override
  public void startPath() {
    Pose2d nearestPos =
        new Pose2d(findNearest(positions).getTranslation(), findNearest(positions).getRotation());

    pose2dArray = new Pose2d[] {nearestPos};
    endRotation = findNearest(positions).getRotation();

    super.startPath();
  }
}
