package frc.robot.sensors.odometry;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants.CameraConstants;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.sensors.apriltag.AprilTagVision;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.periodic.PeriodicBase;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class RobotOdometry extends PeriodicBase {
  DriveSubsystem driveSubsystem;
  Gyro gyro;
  public static RobotOdometry instance;
  private AprilTagVision[] aprilTagVisions;

  public RobotOdometry(DriveSubsystem driveSubsystem, Gyro gyro, AprilTagVision[] aprilTagVisions) {
    this.driveSubsystem = driveSubsystem;
    this.aprilTagVisions = aprilTagVisions;
    instance = this;
    this.gyro = gyro;
    SparkOdometryThread.getInstance().start();
  }

  @Override
  public void periodic() {
    for (AprilTagVision aprilTagVision : aprilTagVisions) {
      aprilTagVision.periodic();
    }
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
        new Pose2d(),
        CameraConstants.defaultDriveStandardDev,
        CameraConstants.defaultVisionStandardDev);
  }

  public void addEstimator(String name, SwerveDrivePoseEstimator estimator) {
    odometries.put(name, new OdometryStorage(estimator));
  }

  public void resetGyro(Pose2d newPose) {
    gyro.setOffset(
        gyro.getRawAngleRadians()
            - newPose.getRotation().getRadians()
            + (DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red
                ? Math.PI
                : 0));
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

  public void addVisionEstimate(String estimator, AprilTagVision vision) {
    SwerveDrivePoseEstimator odometry = odometries.get(estimator).estimator;
    Pose2d visionUpdate = vision.getPose().pose().toPose2d();
    if (!(isPoseValid(visionUpdate)
        && vision.isConnected()
        && vision.getPose().tagCount() > 0
        && vision.getPose().ambiguity() < 0.5
        && vision.getPose().pose().getZ() < 0.75)) {
      return;
    }
    if (vision.getPose().tagCount() == 1 && vision.getPose().ambiguity() > 0.3) {
      return;
    }
    double distFactor =
        Math.pow(vision.getPose().averageTagDistance(), 2.0) / vision.getPose().tagCount();
    double xy = 0.02 * distFactor;
    double rot = Double.MAX_VALUE;
    if (vision.getPose().ambiguity() < 0.1 && vision.getPose().tagCount() > 1) {
      rot = 0.06 * distFactor;
    }
    odometry.addVisionMeasurement(
        visionUpdate, vision.getPose().timestamp(), VecBuilder.fill(xy, xy, rot));
  }

  public boolean isPoseValid(Pose2d pose) {
    return FieldConstants.width >= pose.getX()
        && FieldConstants.height >= pose.getY()
        && pose.getX() > 0
        && pose.getY() > 0;
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
