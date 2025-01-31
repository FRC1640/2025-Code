package frc.robot.sensors.apriltag;

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
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends PeriodicBase {
  AprilTagVisionIO io;
  AprilTagVisionIOInputsAutoLogged inputs;
  private String cameraName;
  private double standardDeviation;

  private Map<Integer, PoseObservation> trigPoses = new HashMap<Integer, PoseObservation>();

  public AprilTagVision(AprilTagVisionIO io, CameraConstant cameraConstants) {
    this.io = io;
    cameraName = cameraConstants.name;
    standardDeviation = cameraConstants.standardDevConstant;
    this.inputs = new AprilTagVisionIOInputsAutoLogged();
    for (int i = 1; i <= FieldConstants.tagCount; i++) {
      trigPoses.put(i, new PoseObservation(0, new Pose3d(), 0, 0, 0));
    }
    AlertsManager.addAlert(
        () -> !inputs.connected, "April tag vision disconnected.", AlertType.kError);
  }

  public void calculateTrigResults(Rotation3d gyroRotation) {
    // trig solution
    for (TrigTargetObservation observation : inputs.trigTargetObservations) {
      addTrigResult(observation, gyroRotation);
    }
  }

  public void addTrigResult(TrigTargetObservation observation, Rotation3d gyroRotation) {
    // trig solution
    if (trigPoses.containsKey(observation.fiducialId())
        && observation.timestamp() <= trigPoses.get(observation.fiducialId()).timestamp()) {
      return;
    }

    // TODO get multiple tx's and ty's and average them?

    Logger.recordOutput(
        "AprilTagVision/" + cameraName + "/TrigEstimate/DEBUG/fieldToTarget",
        new Pose3d(
            observation.targetTransform().getTranslation(),
            observation.targetTransform().getRotation()));
    Logger.recordOutput(
        "AprilTagVision/" + cameraName + "/TrigEstimate/DEBUG/origin",
        new Transform3d(0, 0, 0, new Rotation3d()));

    // calculate
    double deltaH = observation.targetTransform().getZ() - io.getCameraDisplacement().getZ();
    double distance2d =
        deltaH
            / Math.tan(
                -observation.ty().getRadians() - io.getCameraDisplacement().getRotation().getY());
    double x = -distance2d * Math.cos(observation.tx().getRadians() - gyroRotation.getZ());
    double y = distance2d * Math.sin(observation.tx().getRadians() - gyroRotation.getZ());
    double z = 0;

    // logs for debugging
    Logger.recordOutput("AprilTagVision/" + cameraName + "/TrigEstimate/DEBUG/transform/x", x);
    Logger.recordOutput("AprilTagVision/" + cameraName + "/TrigEstimate/DEBUG/transform/y", y);
    Logger.recordOutput("AprilTagVision/" + cameraName + "/TrigEstimate/DEBUG/transform/z", z);
    Logger.recordOutput(
        "AprilTagVision/" + cameraName + "/TrigEstimate/DEBUG/transform/xy_sum", x + y);

    // calculate transforms
    Transform3d cameraToTarget = new Transform3d(new Translation3d(x, y, z), new Rotation3d());
    Transform3d fieldToCamera = observation.targetTransform().plus(cameraToTarget.inverse());
    Transform3d fieldToRobot = fieldToCamera.plus(inputs.cameraDisplacement.inverse());

    // convert and record
    Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), gyroRotation);
    trigPoses.put(
        observation.fiducialId(),
        new PoseObservation(observation.timestamp(), robotPose, 0, 1, observation.distance()));
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

  public double getStandardDeviation() {
    return standardDeviation;
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
        "Drive/Odometry/Vision/Camera_" + cameraName + "/TagPoses",
        tagPoses.toArray(Pose3d[]::new));
    // testing
    Rotation2d gyroRotation = RobotOdometry.instance.getPose("Normal").getRotation();
    calculateTrigResults(new Rotation3d(0, 0, gyroRotation.getRadians()));
    Logger.recordOutput(
        "AprilTagVision/" + cameraName + "/TrigEstimate/TagPoses",
        trigPoses.get(inputs.closestTagId).pose());
  }
}
