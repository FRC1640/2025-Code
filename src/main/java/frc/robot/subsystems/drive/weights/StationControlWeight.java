package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.function.DoubleFunction;
import java.util.function.Supplier;
import frc.robot.constants.RobotConstants.AutoAlignConfig;

public class StationControlWeight implements DriveWeight {
  private PIDController velocityController =
      RobotPIDConstants.constructPID(
          RobotPIDConstants.velocityControlPid, "stationVelocityController");

  private DriveSubsystem driveSubsystem;
  private Supplier<Pose2d> getRobotPose;
  private Supplier<Pose2d> getTargetPose;

  public StationControlWeight(
      DriveSubsystem driveSubsystem,
      Supplier<Pose2d> getRobotPose,
      Supplier<Pose2d> getTargetPose) {
    this.driveSubsystem = driveSubsystem;
    this.getRobotPose = getRobotPose;
    this.getTargetPose = getTargetPose;
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    Pose2d robotPose = getRobotPose.get();
    Pose2d targetPose = getTargetPose.get();
    double distance = robotPose.minus(targetPose).getTranslation().getNorm();
    double desiredSpeed = getDesiredVelocity(distance);
    double outputSpeed =
        velocityController.calculate(driveSubsystem.chassisSpeedsMagnitude(), desiredSpeed);
    ChassisSpeeds robotVelocity = driveSubsystem.getChassisSpeeds();
    Rotation2d velocityAngle =
        new Translation2d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond)
            .getAngle();
    return new ChassisSpeeds(
        outputSpeed * velocityAngle.getCos(),
        outputSpeed * velocityAngle.getSin(),
        robotVelocity.omegaRadiansPerSecond);
  }

  private double getDesiredVelocity(double distance) {

    -(s+1) * DriveConstants.maxSpeed * Math.exp(-) + vmax
  }
}
