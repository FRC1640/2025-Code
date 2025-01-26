package frc.robot.sensors.apriltag;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.constants.CameraConstant;
import frc.robot.constants.FieldConstants;
import frc.robot.sensors.apriltag.AprilTagVisionIO.PoseObservation;
import frc.robot.sensors.apriltag.AprilTagVisionIO.TrigTargetObservation;
import java.util.ArrayList;
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

  public ArrayList<PoseObservation> calculateTrigResults(Rotation3d gyroRotation) {
    ArrayList<PoseObservation> results = new ArrayList<>();
    // trig solution
    for (TrigTargetObservation observation : inputs.trigTargetObservations) {
      Transform3d fieldToTarget =
          new Transform3d(
              observation.targetPose().getTranslation(), observation.targetPose().getRotation());
      double x =
          observation.distance()
              * Math.cos(observation.ty().getDegrees())
              * Math.sin(observation.tx().getDegrees());
      double y =
          observation.distance()
              * Math.cos(observation.ty().getDegrees())
              * Math.cos(observation.tx().getDegrees());
      double z = observation.distance() * Math.sin(observation.ty().getDegrees());
      Transform3d cameraToTarget = new Transform3d(new Translation3d(x, y, z), gyroRotation);
      Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
      Transform3d fieldToRobot = fieldToCamera.plus(inputs.cameraDisplacement.inverse());
      Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());
      results.add(
          new PoseObservation(observation.timestamp(), robotPose, 0, 1, observation.distance()));
    }
    return results;
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
