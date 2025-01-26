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
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class RobotOdometry extends PeriodicBase {
  DriveSubsystem driveSubsystem;
  Gyro gyro;
  public static RobotOdometry instance;
  private boolean useAutoApriltags = false;
  public HashMap<String, OdometryStorage> odometries = new HashMap<>();
  public HashMap<String, AprilTagVision> visionMap = new HashMap<>();

  public enum VisionUpdateMode {
    PHOTONVISION,
    TRIG,
    DYNAMIC
  }

  public boolean isUseAutoApriltags() {
    return useAutoApriltags;
  }

  public void setUseAutoApriltags(boolean useAutoApriltags) {
    this.useAutoApriltags = useAutoApriltags;
  }

  public RobotOdometry(DriveSubsystem driveSubsystem, Gyro gyro, AprilTagVision[] cameras) {
    this.driveSubsystem = driveSubsystem;
    instance = this;
    this.gyro = gyro;
    for (AprilTagVision aprilTagVision : cameras) {
      visionMap.put(aprilTagVision.getCameraName(), aprilTagVision);
    }
    SparkOdometryThread.getInstance().start();
    branchEstimator("Main", cameras, VisionUpdateMode.PHOTONVISION);
  }

  @Override
  public void periodic() {
    updateAllOdometries();
  }

  public static SwerveDrivePoseEstimator getDefaultEstimator(Pose2d initalPose) {
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

  public OdometryStorage branchEstimator(
      String name, String[] cameras, VisionUpdateMode visionUpdateMode) {
    OdometryStorage o =
        new OdometryStorage(
            name,
            getDefaultEstimator(new Pose2d()),
            Arrays.stream(cameras).map((x) -> visionMap.get(x)).toArray(AprilTagVision[]::new),
            visionUpdateMode);
    odometries.put(name, o);
    return o;
  }

  public OdometryStorage branchEstimator(
      String name, AprilTagVision[] cameras, VisionUpdateMode visionUpdateMode) {
    OdometryStorage o =
        new OdometryStorage(name, getDefaultEstimator(new Pose2d()), cameras, visionUpdateMode);
    odometries.put(name, o);
    return o;
  }

  public OdometryStorage branchEstimator(
      String name,
      AprilTagVision[] cameras,
      VisionUpdateMode visionUpdateMode,
      OdometryStorage branchFrom) {
    OdometryStorage o =
        new OdometryStorage(
            name,
            getDefaultEstimator(branchFrom.estimator.getEstimatedPosition()),
            cameras,
            visionUpdateMode);
    odometries.put(name, o);
    return o;
  }

  public OdometryStorage branchEstimator(
      String name,
      String[] cameras,
      VisionUpdateMode visionUpdateMode,
      OdometryStorage branchFrom) {
    OdometryStorage o =
        new OdometryStorage(
            name,
            getDefaultEstimator(branchFrom.estimator.getEstimatedPosition()),
            Arrays.stream(cameras).map((x) -> visionMap.get(x)).toArray(AprilTagVision[]::new),
            visionUpdateMode);
    odometries.put(name, o);
    return o;
  }

  public void pruneBranch(OdometryStorage estimator) {
    odometries.remove(estimator.getName());
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

  public void updateAllOdometries() {
    for (var estimator : odometries.values()) {
      updateOdometryWheels(estimator);
      for (AprilTagVision aprilTagVision : estimator.getVisions()) {
        addVisionEstimate(estimator, aprilTagVision);
      }
    }
  }

  public Pose2d getPose(String name) {
    return odometries.get(name).estimator.getEstimatedPosition();
  }

  public void setPose(String name, Pose2d pose) {
    odometries.get(name).estimator.resetPose(pose);
  }

  public void addVisionEstimate(OdometryStorage odometryStorage, AprilTagVision vision) {
    List<Pose2d> robotPoses = new LinkedList<>();
    List<Pose2d> robotPosesAccepted = new LinkedList<>();
    List<Pose2d> robotPosesRejected = new LinkedList<>();
    for (PoseObservation poseObservation : vision.getPoses()) {
      SwerveDrivePoseEstimator odometry = odometryStorage.estimator;
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
      double xy = 0.1 * distFactor;
      double rot = Double.MAX_VALUE;
      if (poseObservation.ambiguity() < 0.05 && poseObservation.tagCount() > 1) {
        rot = 0.06 * distFactor;
      }
      Logger.recordOutput("Drive/Odometry/Vision/" + odometryStorage.getName() + "/xyDev", xy);
      Logger.recordOutput("Drive/Odometry/Vision/" + odometryStorage.getName() + "/rotDev", rot);
      Logger.recordOutput(
          "Drive/Odometry/Vision/" + odometryStorage.getName() + "/distFactor", distFactor);
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

  public void updateOdometryWheels(OdometryStorage odometryStorage) {
    OdometryStorage e = odometryStorage;
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
      Logger.recordOutput(
          "Drive/Odometry/" + odometryStorage.getName(), e.estimator.getEstimatedPosition());
    }
  }
}
