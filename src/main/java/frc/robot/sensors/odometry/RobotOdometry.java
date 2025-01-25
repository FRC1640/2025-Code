package frc.robot.sensors.odometry;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.Robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants.CameraConstants;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.sensors.apriltag.AprilTagVision;
import frc.robot.sensors.apriltag.AprilTagVisionIO.PoseObservation;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.periodic.PeriodicBase;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class RobotOdometry extends PeriodicBase {
  DriveSubsystem driveSubsystem;
  Gyro gyro;
  public static RobotOdometry instance;
  private AprilTagVision[] aprilTagVisions;
  private boolean useAutoApriltags = false;

  public boolean isUseAutoApriltags() {
    return useAutoApriltags;
  }

  public void setUseAutoApriltags(boolean useAutoApriltags) {
    this.useAutoApriltags = useAutoApriltags;
  }

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
      for (AprilTagVision aprilTagVision : aprilTagVisions) {
        addVisionEstimate(estimator, aprilTagVision);
      }
    }
  }

  public void addVisionEstimate(String estimator, AprilTagVision vision) {
    List<Pose2d> robotPoses = new LinkedList<>();
    List<Pose2d> robotPosesAccepted = new LinkedList<>();
    List<Pose2d> robotPosesRejected = new LinkedList<>();
    for (PoseObservation poseObservation : vision.getPoses()) {
      SwerveDrivePoseEstimator odometry = odometries.get(estimator).estimator;
      Pose2d visionUpdate = poseObservation.pose().toPose2d();
      robotPoses.add(visionUpdate);
      if (Robot.getState() == RobotState.DISABLED) {
        continue;
      }
      if (Robot.getState() == RobotState.AUTONOMOUS && !useAutoApriltags) {
        continue;
      }
      if (!(isPoseValid(visionUpdate)
          && vision.isConnected()
          && poseObservation.tagCount() > 0
          && poseObservation.ambiguity() < 0.2
          && Math.abs(poseObservation.pose().getZ()) < 0.75)) {
        robotPosesRejected.add(visionUpdate);
        continue;
      }
      robotPosesAccepted.add(visionUpdate);
      double distFactor =
          Math.pow(poseObservation.averageTagDistance(), 2.0)
              / poseObservation.tagCount()
              * vision.getStandardDeviation();
      double xy = 0.02 * distFactor;
      double rot = Double.MAX_VALUE;
      if (poseObservation.ambiguity() < 0.05 && poseObservation.tagCount() > 1) {
        rot = 0.06 * distFactor;
      }
      Logger.recordOutput("Drive/Odometry/Vision/" + estimator + "/xyDev", xy);
      Logger.recordOutput("Drive/Odometry/Vision/" + estimator + "/rotDev", rot);
      Logger.recordOutput("Drive/Odometry/Vision/" + estimator + "/distFactor", distFactor);
      odometry.addVisionMeasurement(
          visionUpdate, poseObservation.timestamp(), VecBuilder.fill(xy, xy, rot));
    }
    for (Pose2d pose : robotPoses) {
      Logger.recordOutput(
          "Drive/Odometry/Vision/Camera_" + vision.getCameraName() + "/RobotPoses", pose);
    }
    for (Pose2d pose : robotPosesAccepted) {
      Logger.recordOutput(
          "Drive/Odometry/Vision/Camera_" + vision.getCameraName() + "/RobotPosesAccepted", pose);
    }
    for (Pose2d pose : robotPosesRejected) {
      Logger.recordOutput(
          "Drive/Odometry/Vision/Camera_" + vision.getCameraName() + "/RobotPosesRejected", pose);
    }
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
