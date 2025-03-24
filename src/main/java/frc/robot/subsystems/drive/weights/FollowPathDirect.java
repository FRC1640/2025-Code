package frc.robot.subsystems.drive.weights;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.misc.DistanceManager;
import java.util.function.Function;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

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

  @Override
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
        (target.getTranslation().getDistance(robot.getTranslation())
                < Units.inchesToMeters(RobotConstants.RobotDimensions.robotLengthInches) / 2 + 0.1
            && Math.abs(endRotation.minus(robot.getRotation()).getDegrees()) < 2);
    Logger.recordOutput("A_DEBUG/distanceThreshold",
        Units.inchesToMeters(RobotConstants.RobotDimensions.robotLengthInches) / 2 + 0.01);
    Logger.recordOutput("A_DEBUG/distance", target.getTranslation().getDistance(robot.getTranslation()));
    // ChassisSpeeds chassisSpeeds = super.getDriveChassisSpeeds();
    // complete &=
    //     Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond) < 0.005;
    return complete;
  }
}
