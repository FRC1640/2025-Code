package frc.robot.sensors.odometry;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.periodic.PeriodicBase;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class RobotOdometry extends PeriodicBase {
  DriveSubsystem driveSubsystem;
  Gyro gyro;
  public static RobotOdometry instance;

  public RobotOdometry(DriveSubsystem driveSubsystem, Gyro gyro) {
    this.driveSubsystem = driveSubsystem;
    instance = this;
    this.gyro = gyro;
    SparkOdometryThread.getInstance().start();
  }

  @Override
  public void periodic() {
    updateAllOdometries();
  }

  public HashMap<String, OdometryStorage> odometries = new HashMap<>();

  public static SwerveDrivePoseEstimator getDefaultEstimator() {
    return new SwerveDrivePoseEstimator(
        DriveConstants.kinematics,
        new Rotation2d(),
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        },
        new Pose2d());
  }

  public void addEstimator(String name, SwerveDrivePoseEstimator estimator) {
    odometries.put(name, new OdometryStorage(estimator));
  }

  public Pose2d getPose(String estimator) {
    return odometries.get(estimator).estimator.getEstimatedPosition();
  }

  public void setPose(Pose2d pose, String estimator) {
    odometries
        .get(estimator)
        .estimator
        .resetPosition(gyro.getRawAngleRotation2d(), driveSubsystem.getModulePositions(), pose);
  }

  public void setAllPose(Pose2d pose) {
    for (var estimator : odometries.values()) {
      estimator.estimator.resetPosition(
          gyro.getRawAngleRotation2d(), driveSubsystem.getModulePositions(), pose);
    }
  }

  public void updateAllOdometries() {
    for (var estimator : odometries.keySet()) {
      updateOdometryWheels(estimator);
    }
  }

  public void updateOdometryWheels(String estimator) {
    OdometryStorage e = odometries.get(estimator);
    double[] sampleTimestamps = driveSubsystem.getModules()[0].getOdometryTimestamps();
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] =
            driveSubsystem.getModules()[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - e.lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        e.lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }
      // Update gyro angle
      if (gyro.isTrustworthy()) {
        // Use the real gyro angle
        e.rawGyroRotation = gyro.getOdometryPositions()[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = DriveConstants.kinematics.toTwist2d(moduleDeltas);
        e.rawGyroRotation = e.rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      e.estimator.updateWithTime(sampleTimestamps[i], e.rawGyroRotation, modulePositions);
      Logger.recordOutput("Drive/Odometry/" + estimator, e.estimator.getEstimatedPosition());
    }
  }
}
