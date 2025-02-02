package frc.robot.sensors.apriltag;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.CameraConstant;
import frc.robot.constants.FieldConstants;
import frc.robot.sensors.apriltag.AprilTagVisionIO.PoseObservation;
import frc.robot.sensors.apriltag.AprilTagVisionIO.TrigTargetObservation;
import frc.robot.sensors.odometry.RobotOdometry;
import frc.robot.util.alerts.AlertsManager;
import frc.robot.util.periodic.PeriodicBase;
import java.util.ArrayList;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends PeriodicBase {
  AprilTagVisionIO io;
  AprilTagVisionIOInputsAutoLogged inputs;
  private String cameraName;
  public final double standardDeviation;

  public AprilTagVision(AprilTagVisionIO io, CameraConstant cameraConstants) {
    this.io = io;
    cameraName = cameraConstants.name;
    this.inputs = new AprilTagVisionIOInputsAutoLogged();
    this.standardDeviation = cameraConstants.standardDevConstant;
    AlertsManager.addAlert(
        () -> !inputs.connected, "April tag vision disconnected.", AlertType.kError);
  }

  public double getStandardDeviation() {
    return standardDeviation;
  }

  public double getPhotonDistFactor(PoseObservation observation) {
    double distFactor =
        Math.pow(observation.minimumTagDistance(), 2.0)
            / observation.tagCount()
            * getStandardDeviation();
    Logger.recordOutput("AprilTagVision/" + cameraName + "/Stddevs/distFactorPhoton", distFactor);
    return distFactor;
  }

  public double getPhotonXyStdDev(PoseObservation observation) {
    double xyStdDev = 0.1 * getPhotonDistFactor(observation);
    Logger.recordOutput("AprilTagVision/" + cameraName + "/Stddevs/xyStdDevPhoton", xyStdDev);
    return xyStdDev;
  }

  public double getPhotonRotStdDev(PoseObservation observation) {
    double rot = Double.MAX_VALUE;
    if (getRotationValidPhotonObservation(observation)) {
      rot = 0.06 * getPhotonDistFactor(observation);
    }
    Logger.recordOutput("AprilTagVision/" + cameraName + "/Stddevs/rotPhoton", rot);
    return rot;
  }

  public boolean getRotationValidPhotonObservation(PoseObservation observation) {
    return (observation.ambiguity() < 0.05 && observation.tagCount() > 1);
  }

  public double getTrigDistFactor(PoseObservation observation) {
    double distFactor =
        Math.pow(observation.averageTagDistance(), 2)
            / observation.tagCount()
            * getStandardDeviation();
    Logger.recordOutput("AprilTagVision/" + cameraName + "/Stddevs/distFactorTrig", distFactor);
    return distFactor;
  }

  public double getTrigXyStdDev(PoseObservation observation) {
    double xy = 0.01 * getTrigDistFactor(observation);
    Logger.recordOutput("AprilTagVision/" + cameraName + "/Stddevs/xyStdDevTrig", xy);
    return xy;
  }

  public Optional<PoseObservation> getTrigResult(Rotation2d gyroRotation) {
    // trig solution
    ArrayList<PoseObservation> trigPoses = new ArrayList<>();
    if (inputs.trigTargetObservations.length == 0) {
      return Optional.empty();
    }
    for (TrigTargetObservation observation : inputs.trigTargetObservations) {
      PoseObservation result = calculateTrigResult(observation, gyroRotation);
      trigPoses.add(result);
    }
    // double bestDist = Double.MAX_VALUE;
    // Pose2d bestPose = new Pose2d();

    // calculate averages
    double averageX = 0;
    double averageY = 0;
    double total = 0;
    double averageDistance = 0;
    double minimumDistance = Double.MAX_VALUE;
    for (PoseObservation poseObservation : trigPoses) {
      // if (poseObservation.averageTagDistance() < bestDist) {
      // bestDist = poseObservation.averageTagDistance();
      // bestPose = poseObservation.pose().toPose2d();
      // }
      double distFactor = (1 / Math.pow(poseObservation.minimumTagDistance(), 2));
      averageX += distFactor * poseObservation.pose().getX();
      averageY += distFactor * poseObservation.pose().getY();
      total += distFactor;
      averageDistance += poseObservation.minimumTagDistance();
      if (poseObservation.minimumTagDistance() < minimumDistance) {
        minimumDistance = poseObservation.minimumTagDistance();
      }
    }
    averageX /= total;
    averageY /= total;
    averageDistance /= trigPoses.size();
    Pose2d averagePose = new Pose2d(averageX, averageY, gyroRotation);

    PoseObservation observation =
        new PoseObservation(
            trigPoses.get(trigPoses.size() - 1).timestamp(),
            new Pose3d(averagePose),
            0,
            trigPoses.size(),
            averageDistance,
            minimumDistance);
    // Logger.recordOutput(cameraName, null);
    // return Optional.of(bestPose);
    return Optional.of(observation);
  }

  public PoseObservation calculateTrigResult(
      TrigTargetObservation observation, Rotation2d gyroRotation) {
    // Get camera displacement details
    Transform3d cameraDisplacement = inputs.cameraDisplacement;
    Translation3d cameraToRobot = cameraDisplacement.getTranslation();
    Rotation3d cameraRotation = cameraDisplacement.getRotation();

    // Calculate vertical offset between camera and tag
    double deltaZ =
        FieldConstants.aprilTagLayout.getTagPose(observation.fiducialId()).get().getZ()
            - cameraToRobot.getZ();

    // get distance
    double distance2d = observation.cameraToTarget().getTranslation().toTranslation2d().getNorm();

    double tx = -observation.tx().getRadians();

    // Calculate local translation in camera's coordinate system
    Translation3d cameraToTagCameraFrame =
        new Translation3d(distance2d * Math.cos(tx), distance2d * Math.sin(tx), -deltaZ);

    // Rotate through coordinate systems:
    Translation3d cameraToTagFieldFrame =
        cameraToTagCameraFrame
            .rotateBy(cameraRotation)
            .rotateBy(new Rotation3d(0, 0, gyroRotation.getRadians()));

    // Calculate camera position in field coordinates
    Translation3d fieldToCamera =
        FieldConstants.aprilTagLayout
            .getTagPose(observation.fiducialId())
            .get()
            .getTranslation()
            .minus(cameraToTagFieldFrame);

    // Convert camera displacement to field coordinates
    Translation3d cameraDisplacementField =
        cameraToRobot.rotateBy(new Rotation3d(0, 0, gyroRotation.getRadians()));

    // Calculate robot position in field coordinates
    Translation3d fieldToRobotTranslation = fieldToCamera.minus(cameraDisplacementField);

    // Create final pose using gyro-reported rotation
    Pose3d robotPose =
        new Pose3d(fieldToRobotTranslation, new Rotation3d(0, 0, gyroRotation.getRadians()));

    return new PoseObservation(
        observation.timestamp(),
        robotPose,
        0,
        1,
        observation.cameraToTarget().getTranslation().getNorm(),
        observation.cameraToTarget().getTranslation().getNorm());
  }

  public PoseObservation[] getPhotonResults() {
    return inputs.photonPoseObservations;
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  public String getCameraName() {
    return cameraName;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("AprilTagVision/" + cameraName, inputs);
    ArrayList<Pose3d> tagPoses = new ArrayList<>();
    for (int i = 0; i < inputs.tagIds.length; i++) {
      Optional<Pose3d> pose = FieldConstants.aprilTagLayout.getTagPose(inputs.tagIds[i]);
      if (pose.isPresent()) {
        tagPoses.add(pose.get());
      }
    }
    Logger.recordOutput(
        "AprilTagVision/" + cameraName + "/TagPoses", tagPoses.toArray(Pose3d[]::new));
    // // testing
    Rotation2d gyroRotation = RobotOdometry.instance.getPose("Main").getRotation();
    Optional<PoseObservation> robotTrig = getTrigResult(gyroRotation);

    PoseObservation robotPose = robotTrig.orElse(null);
    Logger.recordOutput(
        "AprilTagVision/" + cameraName + "/TrigEstimate/RobotPose",
        robotPose == null ? new Pose2d() : robotPose.pose().toPose2d());
  }
}
