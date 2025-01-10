package frc.robot.sensors.apriltag;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagVisionIOPhotonvision implements AprilTagVisionIO {
  private final PhotonCamera camera; // the camera
  private final Transform3d cameraDisplacement; // represents position of camera relative to robot

  public AprilTagVisionIOPhotonvision(
      String name, Transform3d cameraDisplacement) { // name should match camera "nickname"
    this.camera = new PhotonCamera(name);
    this.cameraDisplacement = cameraDisplacement;
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    inputs.connected = camera.isConnected(); // easy to update
    Set<Short> tagIds = new HashSet<>(); // will be filled in in loop and sent to inputs
    List<PoseObservation> poseObservations =
        new LinkedList<>(); // will be sent to io after loop. Why linked?
    for (PhotonPipelineResult result : camera.getAllUnreadResults()) { // monitor call frequency
      if (result.hasTargets()) { // if it has a target? Must mean if it has an apriltag in view
        inputs.latestTargetObservation =
            new AprilTagObservation(
                new Rotation2d(
                    result
                        .getBestTarget()
                        .getYaw()), // best target seems to rely on some sort of configuration
                new Rotation2d(
                    result
                        .getBestTarget()
                        .getPitch())); // sends tx (yaw) and ty (pitch) to the inputs
      } else {
        inputs.latestTargetObservation =
            new AprilTagObservation(
                new Rotation2d(),
                new Rotation2d()); // mechanical advantage did this, but maybe there's a better way
      }
      if (result.multitagResult.isPresent()) { // if the current result can see multiple tags?
        MultiTargetPNPResult multitag = result.multitagResult.get();
        Transform3d cameraTransform = multitag.estimatedPose.best;
        Transform3d robotTransform = cameraTransform.plus(cameraDisplacement.inverse());
        Pose3d robotPose =
            new Pose3d(robotTransform.getTranslation(), robotTransform.getRotation());
        double totalTagDistance = 0;
        for (PhotonTrackedTarget target : result.getTargets()) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }
        tagIds.addAll(multitag.fiducialIDsUsed);
        poseObservations.add(
            new PoseObservation(
                multitag.estimatedPose.ambiguity,
                result.getTimestampSeconds(),
                robotPose,
                multitag.fiducialIDsUsed.size(),
                totalTagDistance / result.targets.size()));
      } else if (!result.targets.isEmpty()) {
        PhotonTrackedTarget target = result.targets.get(0);
        Optional<Pose3d> tagPose =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)
                .getTagPose(target.fiducialId); // move to constants?
        if (tagPose.isPresent()) {
          Transform3d tagTransform =
              new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
          Transform3d cameraToTag = target.bestCameraToTarget;
          Transform3d cameraTransform = tagTransform.plus(cameraToTag.inverse());
          Transform3d robotTransform = cameraTransform.plus(cameraDisplacement.inverse());
          Pose3d robotPose =
              new Pose3d(robotTransform.getTranslation(), robotTransform.getRotation());
          tagIds.add((short) target.fiducialId);
          poseObservations.add(
              new PoseObservation(
                  target.poseAmbiguity,
                  result.getTimestampSeconds(),
                  robotPose,
                  1,
                  cameraToTag.getTranslation().getNorm()));
        }
      }
      inputs.poseObservations = new PoseObservation[poseObservations.size()];
      for (int i = 0; i < poseObservations.size(); i++) {
        inputs.poseObservations[i] = poseObservations.get(i);
      }
      int index = 0;
      for (Short id : tagIds) {
        inputs.tagIds[index] = id;
        index++;
      }
    }
  }
}
