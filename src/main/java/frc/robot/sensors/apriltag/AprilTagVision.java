package frc.robot.sensors.apriltag;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.CameraConstant;
import frc.robot.constants.FieldConstants;
import frc.robot.sensors.apriltag.AprilTagVisionIO.PoseObservation;
import frc.robot.util.periodic.PeriodicBase;
import frc.robot.util.alerts.AlertsManager;
import java.util.ArrayList;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends PeriodicBase {
  AprilTagVisionIO io;
  AprilTagVisionIOInputsAutoLogged inputs;
  private String cameraName;
  private double standardDeviation;

  public AprilTagVision(AprilTagVisionIO io, CameraConstant cameraConstants) {
    this.io = io;
    cameraName = cameraConstants.name;
    standardDeviation = cameraConstants.standardDevConstant;
    this.inputs = new AprilTagVisionIOInputsAutoLogged();
    AlertsManager.addAlert(
        () -> !inputs.connected, "April tag vision disconnected.", AlertType.kError);
  }

  public Rotation2d getTx() {
    return inputs.latestTargetObservation.tx();
  }

  public Rotation2d getTy() {
    return inputs.latestTargetObservation.ty();
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  public PoseObservation[] getPoses() {
    return inputs.poseObservations;
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
  }
}
