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

  public double getDistFactor(PoseObservation observation) {
    double distFactor =
        Math.pow(observation.averageTagDistance(), 2.0)
            / observation.tagCount()
            * getStandardDeviation();
    Logger.recordOutput("AprilTagVision/" + cameraName + "/Stddevs/distFactor", distFactor);
    return distFactor;
  }

  public double getXyStdDev(PoseObservation observation) {
    double xyStdDev = 0.1 * getDistFactor(observation);
    Logger.recordOutput("AprilTagVision/" + cameraName + "/Stddevs/xyStdDev", xyStdDev);
    return xyStdDev;
  }

  public double getRotStdDev(PoseObservation observation) {
    double rot = Double.MAX_VALUE;
    if (observation.ambiguity() < 0.05 && observation.tagCount() > 1) {
      rot = 0.06 * getDistFactor(observation);
    }
    Logger.recordOutput("AprilTagVision/" + cameraName + "/Stddevs/rot", rot);
    return rot;
  }

  public Optional<Pose2d> calculateTrigResults(Rotation3d gyroRotation) {
    // trig solution
    ArrayList<PoseObservation> trigPoses = new ArrayList<>();
    if (inputs.trigTargetObservations.length == 0) {
      return Optional.empty();
    }
    for (TrigTargetObservation observation : inputs.trigTargetObservations) {
      trigPoses.add(calculateTrigResult(observation, gyroRotation));
    }
    // double bestDist = Double.MAX_VALUE;
    // Pose2d bestPose = new Pose2d();
    double averageX = 0;
    double averageY = 0;
    double total = 0;
    for (PoseObservation poseObservation : trigPoses) {
      // if (poseObservation.averageTagDistance() < bestDist) {
      // bestDist = poseObservation.averageTagDistance();
      // bestPose = poseObservation.pose().toPose2d();
      // }
      double distFactor = (1 / Math.pow(poseObservation.averageTagDistance(), 2));
      averageX += distFactor * poseObservation.pose().getX();
      averageY += distFactor * poseObservation.pose().getY();
      total += distFactor;
    }
    averageX /= total;
    averageY /= total;
    Pose2d averagePose = new Pose2d(averageX, averageY, gyroRotation.toRotation2d());
    // Logger.recordOutput(cameraName, null);
    // return Optional.of(bestPose);
    return Optional.of(averagePose);
  }

  public PoseObservation calculateTrigResult(
      TrigTargetObservation observation, Rotation3d gyroRotation) {
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
        cameraToTagCameraFrame.rotateBy(cameraRotation).rotateBy(gyroRotation);

    // Calculate camera position in field coordinates
    Translation3d fieldToCamera =
        FieldConstants.aprilTagLayout
            .getTagPose(observation.fiducialId())
            .get()
            .getTranslation()
            .minus(cameraToTagFieldFrame);

    // Convert camera displacement to field coordinates
    Translation3d cameraDisplacementField = cameraToRobot.rotateBy(gyroRotation);

    // Calculate robot position in field coordinates
    Translation3d fieldToRobotTranslation = fieldToCamera.minus(cameraDisplacementField);

    // Create final pose using gyro-reported rotation
    Pose3d robotPose = new Pose3d(fieldToRobotTranslation, gyroRotation);

    return new PoseObservation(
        observation.timestamp(),
        robotPose,
        0,
        1,
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
    // testing
    Rotation2d gyroRotation = RobotOdometry.instance.getPose("Main").getRotation();
    Optional<Pose2d> robotTrig =
        calculateTrigResults(new Rotation3d(0, 0, gyroRotation.getRadians()));

    Pose2d robotPose = robotTrig.orElse(null);
    Logger.recordOutput("AprilTagVision/" + cameraName + "/TrigEstimate/RobotPose", robotPose);
  }
}
