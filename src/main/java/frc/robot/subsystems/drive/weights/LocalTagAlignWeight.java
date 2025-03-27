package frc.robot.subsystems.drive.weights;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.sensors.gyro.Gyro;

public class LocalTagAlignWeight implements DriveWeight {
  private Supplier<Pose2d> tagPose;
  private Supplier<Pose2d> robotPose;
  private Gyro gyro;
  @Override
  public ChassisSpeeds getSpeeds() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getSpeeds'");
  }

}