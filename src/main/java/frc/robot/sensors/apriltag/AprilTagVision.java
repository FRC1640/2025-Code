package frc.robot.sensors.apriltag;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.CameraConstant;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants.RobotDimensions;
import frc.robot.sensors.apriltag.AprilTagVisionIO.PoseObservation;
import frc.robot.sensors.apriltag.AprilTagVisionIO.TrigTargetObservation;
import frc.robot.util.alerts.AlertsManager;
import frc.robot.util.periodic.PeriodicBase;
import java.util.ArrayList;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends PeriodicBase {
  AprilTagVisionIO io;
  AprilTagVisionIOInputsAutoLogged inputs;
  private String cameraName;
  private String displayName;
  public final double standardDeviation;
  private Transform3d cameraTransform;

  private Optional<Translation2d> lastLocalVector = Optional.empty();
  private Optional<Translation2d> localVector = Optional.empty();
  private int staleCount = 0;
  private int staleThreshold = 3;
  int lastID = -1;
  int idToUse = -1;
  Optional<Translation2d> output = Optional.empty();

  public AprilTagVision(AprilTagVisionIO io, CameraConstant cameraConstants) {
    this.io = io;
    cameraName = cameraConstants.networkName;
    displayName = cameraConstants.displayName;
    this.inputs = new AprilTagVisionIOInputsAutoLogged();
    this.standardDeviation = cameraConstants.standardDevConstant;
    this.cameraTransform = cameraConstants.transform;
    AlertsManager.addAlert(
        () -> !inputs.connected, "April tag vision disconnected.", AlertType.kError);
  }

  public Transform3d getCameraTransform() {
    return cameraTransform;
  }

  public double getStandardDeviation() {
    return standardDeviation;
  }

  public double getPhotonDistFactor(PoseObservation observation) {
    double distFactor =
        Math.pow(observation.minimumTagDistance(), 2.0)
            / observation.tagCount()
            * getStandardDeviation();
    Logger.recordOutput("AprilTagVision/" + displayName + "/Stddevs/distFactorPhoton", distFactor);
    return distFactor;
  }

  public double getPhotonXyStdDev(PoseObservation observation) {
    double xyStdDev = 0.25 * getPhotonDistFactor(observation);
    Logger.recordOutput("AprilTagVision/" + displayName + "/Stddevs/xyStdDevPhoton", xyStdDev);
    return xyStdDev;
  }

  public double getPhotonRotStdDev(PoseObservation observation) {
    double rot = Double.MAX_VALUE;
    if (getRotationValidPhotonObservation(observation)) {
      rot = 0.06 * getPhotonDistFactor(observation);
    }
    Logger.recordOutput("AprilTagVision/" + displayName + "/Stddevs/rotPhoton", rot);
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
    Logger.recordOutput("AprilTagVision/" + displayName + "/Stddevs/distFactorTrig", distFactor);
    return distFactor;
  }

  public double getTrigXyStdDev(PoseObservation observation) {
    double xy = 0.25 * getTrigDistFactor(observation);
    Logger.recordOutput("AprilTagVision/" + displayName + "/Stddevs/xyStdDevTrig", xy);
    return xy;
  }

  public Optional<PoseObservation> getTrigResult(Rotation2d gyroRotation) {
    // trig solution
    ArrayList<PoseObservation> trigPoses = new ArrayList<>();
    if (inputs.trigTargetObservations.length == 0) {
      return Optional.empty();
    }
    for (TrigTargetObservation observation : inputs.trigTargetObservations) {
      Translation3d cameraToTag = calculateCameraVector(observation);
      PoseObservation result = calculateTrigResult(observation, cameraToTag, gyroRotation);
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

  private Translation3d calculateCameraVector(TrigTargetObservation observation) {

    // Get camera displacement details
    Translation3d cameraToRobot = inputs.cameraDisplacement.getTranslation();

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

    return cameraToTagCameraFrame;
  }

  private PoseObservation calculateTrigResult(
      TrigTargetObservation observation, Translation3d cameraToTag, Rotation2d gyroRotation) {
    // Get camera displacement details
    Transform3d cameraDisplacement = inputs.cameraDisplacement;
    Translation3d cameraToRobot = cameraDisplacement.getTranslation();
    Rotation3d cameraRotation = cameraDisplacement.getRotation();

    // Rotate through coordinate systems:
    Translation3d cameraToTagFieldFrame =
        cameraToTag
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

  public Optional<Translation2d> getLocalAlignVector() {
    return localVector;
  }

  public void setIDToUse(int id) {
    idToUse = id;
  }

  private Optional<Translation2d> getLocalAlignVector(int id) {
    TrigTargetObservation[] trigObservations = inputs.trigTargetObservations;
    Logger.recordOutput("tagID", id);
    Logger.recordOutput("observationlength", inputs.trigTargetObservations.length);
    if (lastID != id) {
      lastLocalVector = Optional.empty();
      staleCount = 0;
      lastID = id;
    }
    Optional<TrigTargetObservation> observation = Optional.empty();
    for (int i = 0; i <= trigObservations.length - 1; i++) {
      if (trigObservations[i].fiducialId() == id) {
        observation = Optional.of(trigObservations[i]);
        break;
      }
    }
    if (observation.isPresent()) {
      // calculate camera vector
      Translation3d vector = calculateCameraVector(observation.get());
      // update local vectors
      Translation2d frontToTag =
          inputs
              .cameraDisplacement
              .getTranslation()
              .plus(vector)
              .toTranslation2d()
              .minus(new Translation2d(RobotDimensions.robotLengthLocalAlign / 2, 0));
      output = Optional.of(frontToTag);
      lastLocalVector = output;
      return output;
    } else if (lastLocalVector.isPresent()) {
      output = Optional.empty();
      staleCount++;
      if (staleCount > staleThreshold) {
        lastLocalVector = Optional.empty();
        staleCount = 0;
      }
      return lastLocalVector;
    } else {
      output = Optional.empty();
      return Optional.empty();
    }
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

  public String getDisplayName() {
    return displayName;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("AprilTagVision/" + displayName, inputs);
    ArrayList<Pose3d> tagPoses = new ArrayList<>();
    for (int i = 0; i < inputs.tagIds.length; i++) {
      Optional<Pose3d> pose = FieldConstants.aprilTagLayout.getTagPose(inputs.tagIds[i]);
      if (pose.isPresent()) {
        tagPoses.add(pose.get());
      }
    }
    localVector = getLocalAlignVector(idToUse);
    Logger.recordOutput(
        "Sensors/AprilTagVision/" + displayName + "/TagPoses", tagPoses.toArray(Pose3d[]::new));
  }
}
