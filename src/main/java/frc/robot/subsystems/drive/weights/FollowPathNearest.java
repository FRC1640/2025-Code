package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.RobotConstants.AutoAlignConfig;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.util.tools.DistanceManager;
import java.util.function.Function;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class FollowPathNearest extends FollowPath {
  Pose2d[] positions;
  Function<Pose2d, Pose2d> poseFunction;
  Supplier<Pose2d> trajSetPoint;
  boolean autoAligned = false;

  public FollowPathNearest(
      Supplier<Pose2d> robotPose,
      Gyro gyro,
      Pose2d[] positions,
      double maxLinearVelocity,
      double maxLinearAcceleration,
      double maxAngularVelocity,
      double maxAngularAcceleration,
      double rotation,
      Supplier<Pose2d> setPointPosition,
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
    trajSetPoint = setPointPosition;

    new Trigger(() -> isNearSetpoint() && controllerButtonDown.getAsBoolean())
        .onFalse(new InstantCommand(() -> restartPath()));
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

  public void restartPath() {
    stopPath();

    startPath();
  }

  @Override
  public void startPath() {
    Pose2d nearestPos =
        new Pose2d(
            findNearest(positions).getTranslation(),
            findNearest(positions).getRotation().unaryMinus());

    pose2dArray = new Pose2d[] {nearestPos};
    endRotation = findNearest(positions).getRotation();

    super.startPath();
  }

  public boolean isNearSetpoint() {
    autoAligned =
        (trajSetPoint.get().getTranslation().getDistance(robotPose.get().getTranslation())
            < AutoAlignConfig.maxDistanceFromTarget);
    Logger.recordOutput("Debug/autoAligned", autoAligned);
    Logger.recordOutput("Debug/robotPose", robotPose.get());
    Logger.recordOutput("Debug/trajSetPoint", trajSetPoint.get());
    return autoAligned;
  }

  public Pose2d getTrajectorySetPoint() {
    return PathplannerWeight.setpoint;
  }
}
