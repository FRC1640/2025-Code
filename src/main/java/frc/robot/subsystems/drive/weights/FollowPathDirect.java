package frc.robot.subsystems.drive.weights;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.misc.DistanceManager;
import java.util.function.Function;
import java.util.function.Supplier;

public class FollowPathDirect extends FollowPath {
  private Supplier<Pose2d[]> positions;
  private Function<Pose2d, Pose2d> poseFunction;
  private PIDController rotPid =
      RobotPIDConstants.constructPID(RobotPIDConstants.angleFollowPath, "FollowPathDirect");

  public FollowPathDirect(
      Supplier<Pose2d[]> positions,
      Function<Pose2d, Pose2d> poseFunction,
      Gyro gyro,
      Supplier<Pose2d> robotPose,
      PathConstraints constraints,
      DriveSubsystem driveSubsystem) {
    super(robotPose, gyro, positions.get(), constraints, null, driveSubsystem);
    this.positions = positions;
    pose2dArray = new Pose2d[] {findNearest(positions.get())};
    endRotation = findNearest(positions.get()).getRotation().minus(new Rotation2d(Math.PI));
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
    // create target
    Pose2d nearest = findNearest(positions.get());
    Pose2d end =
        new Pose2d(
            nearest.getTranslation(),
            robotPose.get().getRotation().rotateBy(new Rotation2d(Math.PI)));
    // override rotation
    endRotation = findNearest(positions.get()).getRotation();
    pose2dArray = new Pose2d[] {end};
    super.startPath();
  }
}
