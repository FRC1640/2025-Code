package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.sensors.gyro.Gyro;
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
            findNearest(this.positions).getRotation().unaryMinus());

    pose2dArray = new Pose2d[] {nearestPos};
    endRotation = findNearest(this.positions).getRotation();

    super.startPath();
  }
}
