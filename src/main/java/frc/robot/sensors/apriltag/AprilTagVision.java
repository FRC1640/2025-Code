package frc.robot.sensors.apriltag;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.sensors.apriltag.AprilTagVisionIO.PoseObservation;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision {
  AprilTagVisionIO io;
  AprilTagVisionIOInputsAutoLogged inputs;
  private String cameraName;

  public AprilTagVision(AprilTagVisionIO io, String cameraName) {
    this.io = io;
    this.cameraName = cameraName;
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

  public boolean getDisconnected() {
    return !inputs.connected;
  }

  public PoseObservation getPose() {
    return inputs.poseObservations[0];
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ApriltagVision/" + cameraName, inputs);
  }
}
