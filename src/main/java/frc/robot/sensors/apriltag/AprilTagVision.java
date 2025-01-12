package frc.robot.sensors.apriltag;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.CameraConstant;
import frc.robot.constants.FieldConstants;
import frc.robot.sensors.apriltag.AprilTagVisionIO.PoseObservation;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision {
  AprilTagVisionIO io;
  AprilTagVisionIOInputsAutoLogged inputs;
  private String cameraName;
  private double standardDeviation;

  public AprilTagVision(AprilTagVisionIO io, CameraConstant cameraConstants) {
    this.io = io;
    cameraName = cameraConstants.name;
    standardDeviation = cameraConstants.standardDevConstant;
    this.inputs = new AprilTagVisionIOInputsAutoLogged();
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

  public PoseObservation[] getPose() {
    return inputs.poseObservations;
  }

  public double getStandardDeviation() {
    return standardDeviation;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("AprilTagVision/" + cameraName, inputs);
    for (int i = 0; i < inputs.tagIds.length; i++) {
      Optional<Pose3d> pose = FieldConstants.aprilTagLayout.getTagPose(inputs.tagIds[i]);
      if (pose.isPresent()) {
        Logger.recordOutput("AprilTagVision/TagPoses", pose.get());
      }
    }
  }
}
