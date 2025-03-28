package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.util.helpers.AutoAlignHelper;
import java.util.function.Supplier;

public class LocalTagAlignWeight implements DriveWeight {
  private Supplier<Pose2d> targetPose;
  private Supplier<Pose2d> robotPose;
  private Gyro gyro;

  @Override
  public ChassisSpeeds getSpeeds() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getSpeeds'");
  }

  public int getTargetTagId() {
    return AutoAlignHelper.getAutoalignTagId(targetPose.get()).ID;
  }
}
