package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.subsystems.drive.commands.AutoAlignHelper;
import java.util.function.Supplier;

public class DriveToPointWeight implements DriveWeight {
  private Supplier<Pose2d> robotPose;
  private Supplier<Pose2d> targetPose;
  private Gyro gyro;
  AutoAlignHelper autoAlignHelper = new AutoAlignHelper();

  public DriveToPointWeight(Supplier<Pose2d> robotPose, Supplier<Pose2d> targetPose, Gyro gyro) {
    this.robotPose = robotPose;
    this.targetPose = targetPose;
    this.gyro = gyro;
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    return autoAlignHelper.getPoseSpeeds(robotPose.get(), targetPose.get(), gyro);
  }
}
