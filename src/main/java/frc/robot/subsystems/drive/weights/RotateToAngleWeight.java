package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.constants.RobotPIDConstants;
import java.util.function.Supplier;

public class RotateToAngleWeight implements DriveWeight {
  private Supplier<Pose2d> robotPose;
  private Supplier<Rotation2d> angle;
  private PIDController rotPid =
      RobotPIDConstants.constructPID(RobotPIDConstants.rotateToAnglePIDRadians);

  public RotateToAngleWeight(Supplier<Pose2d> robotPose, Supplier<Rotation2d> angle) {
    this.robotPose = robotPose;
    this.angle = angle;
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    double pid = rotPid.calculate(robotPose.get().getRotation().minus(angle.get()).getRadians(), 0);
    pid = MathUtil.clamp(pid, -1, 1);
    pid *= DriveConstants.maxOmega;
    return new ChassisSpeeds(0, 0, pid);
  }
}
