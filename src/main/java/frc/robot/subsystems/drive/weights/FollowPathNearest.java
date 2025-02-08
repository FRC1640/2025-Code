package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.util.tools.DistanceManager;
import java.util.function.Supplier;

public class FollowPathNearest extends FollowPath {
  Pose2d[] positions;

  public FollowPathNearest(
      Supplier<Pose2d> robotPose,
      Gyro gyro,
      Pose2d[] positions,
      double maxLinearVelocity,
      double maxLinearAcceleration,
      double maxAngularVelocity,
      double maxAngularAcceleration) {
    super(
        robotPose,
        gyro,
        null,
        maxLinearVelocity,
        maxLinearAcceleration,
        maxAngularVelocity,
        maxAngularVelocity,
        Rotation2d.fromDegrees(maxAngularAcceleration));
    this.positions = positions;
    pose2dArray = new Pose2d[] {findNearest(this.positions)};
    endRotation = findNearest(this.positions).getRotation();
  }

  private Pose2d findNearest(Pose2d[] pos) {
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
