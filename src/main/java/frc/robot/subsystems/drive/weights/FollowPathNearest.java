package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.util.tools.DistanceManager;
import java.util.ArrayList;
import java.util.function.Function;
import java.util.function.Supplier;

public class FollowPathNearest extends FollowPath {
  Pose2d[] positions;
  Function<Pose2d, Pose2d> poseFunction;
  public boolean atPoint = false;

  public FollowPathNearest(
      Supplier<Pose2d> robotPose,
      Gyro gyro,
      Pose2d[] positions,
      double maxLinearVelocity,
      double maxLinearAcceleration,
      double maxAngularVelocity,
      double maxAngularAcceleration,
      double rotation) {
    super(
        robotPose,
        gyro,
        null,
        maxLinearVelocity,
        maxLinearAcceleration,
        maxAngularVelocity,
        maxAngularAcceleration,
        Rotation2d.fromDegrees(rotation));
    this.positions = positions;
    pose2dArray = new Pose2d[] {findNearest(this.positions)};
    endRotation = findNearest(this.positions).getRotation();
  }

  public FollowPathNearest(
      Supplier<Pose2d> robotPose,
      Gyro gyro,
      Pose2d[] positions,
      double maxLinearVelocity,
      double maxLinearAcceleration,
      double maxAngularVelocity,
      double maxAngularAcceleration,
      double rotation,
      Function<Pose2d, Pose2d> poseFunction) {
    super(
        robotPose,
        gyro,
        null,
        maxLinearVelocity,
        maxLinearAcceleration,
        maxAngularVelocity,
        maxAngularAcceleration,
        Rotation2d.fromDegrees(rotation));
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
        new Pose2d(
            findNearest(this.positions).getX(),
            findNearest(this.positions).getY(),
            (findNearest(pose2dArray).getRotation()));
    ArrayList<Pose2d> posesList = new ArrayList<Pose2d>();
    posesList.add(robotPose.get());
    // if (robotPose.get().getTranslation().getDistance(nearestPos.getTranslation())
    //     > AutoAlignConstants.requiredDistanceForMidpoint) {
    //   Pose2d midpoint = new Pose2d(nearestPos.getX(), nearestPos.getY(),
    // nearestPos.getRotation());
    //   midpoint = DistanceManager.addRotatedDim(nearestPos, 2.0, new Rotation2d(0.00001, 0.0001));
    //   posesList.add(midpoint);
    // }

    posesList.add(nearestPos);

    pose2dArray = posesList.toArray(new Pose2d[pose2dArray.length - 1]);
    endRotation = findNearest(this.positions).getRotation();

    super.startPath();
  }
}
