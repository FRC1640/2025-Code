package frc.robot.subsystems.drive.weights;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.sensors.odometry.RobotOdometry;
import frc.robot.util.helpers.AutoAlignHelper;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.constants.RobotPIDConstants;

public class LocalTagAlignWeight implements DriveWeight {
  private Supplier<Translation2d> aprilTagVector = () -> RobotOdometry.getAverageLocalAlignVector();
  private Supplier<AprilTag> targetTag;
  private Supplier<Pose2d> targetPose;
  private Gyro gyro;
  // TODO Trapezoidal Constraints???????? 

  public LocalTagAlignWeight(Supplier<Pose2d> targetPose, Gyro gyro) {
    this.gyro = gyro;
    this.targetPose = targetPose;
    this.targetTag = () -> AutoAlignHelper.getAutoalignTagId(targetPose.get());
  }


  @Override
  public ChassisSpeeds getSpeeds() {
    return AutoAlignHelper.getLocalAlignSpeedsLine();
  }

}
