package frc.robot.subsystems.drive.weights;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.misc.DistanceManager;
import java.util.function.Function;
import java.util.function.Supplier;

public class FollowPathNearest extends FollowPath {
  Supplier<Pose2d[]> positions;
  Function<Pose2d, Pose2d> poseFunction;
  private PIDController rotPid =
      RobotPIDConstants.constructPID(RobotPIDConstants.angleFollowPath, "FollowPathNearest");

  public FollowPathNearest(
      Supplier<Pose2d> robotPose,
      Gyro gyro,
      Supplier<Pose2d[]> positions,
      PathConstraints pathConstraints,
      Function<Pose2d, Pose2d> poseFunction,
      DriveSubsystem driveSubsystem) {
    super(robotPose, gyro, null, pathConstraints, null, driveSubsystem);
    this.positions = positions;
    pose2dArray = new Pose2d[] {findNearest(this.positions.get())};
    endRotation = findNearest(this.positions.get()).getRotation();
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
            findNearest(positions.get()).getTranslation(),
            findNearest(positions.get()).getRotation());

    double v = nearestPos.getTranslation().getDistance(robotPose.get().getTranslation());
    Rotation2d angle =
        nearestPos.getTranslation().minus(robotPose.get().getTranslation()).getAngle();

    double midPointLength = Math.abs(angle.getCos() * v);

    Pose2d midPoint =
        DistanceManager.addRotatedDim(nearestPos, midPointLength * 0.5, nearestPos.getRotation());

    pose2dArray = new Pose2d[] {midPoint, nearestPos};
    endRotation = findNearest(positions.get()).getRotation();

    PathplannerWeight.overrideRotation(() -> omegaOverride(() -> nearestPos.getRotation()));

    super.startPath();
  }

  @Override
  public void stopPath() {
    PathplannerWeight.clearRotationOverride();
    super.stopPath();
  }

  public double omegaOverride(Supplier<Rotation2d> angle) {
    double pid = rotPid.calculate(robotPose.get().getRotation().minus(angle.get()).getRadians(), 0);
    pid = MathUtil.clamp(pid, -1, 1);
    // pid *= DriveConstants.maxOmega;
    return pid;
  }
}
