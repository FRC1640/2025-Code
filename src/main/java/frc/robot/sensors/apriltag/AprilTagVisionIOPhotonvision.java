package frc.robot.sensors.apriltag;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.CameraConstant;
import frc.robot.constants.FieldConstants;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagVisionIOPhotonvision implements AprilTagVisionIO {
  protected final PhotonCamera camera; // the camera
  protected final Transform3d cameraDisplacement; // represents position of camera relative to robot

  public AprilTagVisionIOPhotonvision(
      CameraConstant constant) { // name should match camera "nickname"
    this.camera = new PhotonCamera(constant.name);
    this.cameraDisplacement = constant.transform;
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    inputs.cameraDisplacement = cameraDisplacement;
    inputs.connected = camera.isConnected();

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    List<TrigTargetObservation> trigObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation
      if (result.hasTargets()) {
        PhotonTrackedTarget closestTarget = result.getBestTarget();
        double closestDistance = Integer.MAX_VALUE;
        for (PhotonTrackedTarget target : result.getTargets()) {
          Optional<Pose3d> targetPose = FieldConstants.aprilTagLayout.getTagPose(target.fiducialId);
          if (targetPose.isPresent()) {
            double targetDistance = targetPose.get().getTranslation().getNorm();
            if (targetDistance < closestDistance) {
              closestDistance = targetDistance;
              closestTarget = target;
            }
          }
        }
        var closestTargetPose = FieldConstants.aprilTagLayout.getTagPose(closestTarget.fiducialId);
        trigObservations.add(
            new TrigTargetObservation(
                Rotation2d.fromDegrees(closestTarget.getYaw()),
                Rotation2d.fromDegrees(closestTarget.getPitch()),
                closestDistance,
                closestTargetPose.isPresent()
                    ? closestTargetPose.get()
                    : FieldConstants.aprilTagLayout
                        .getTagPose(result.getBestTarget().fiducialId)
                        .get(),
                result.getTimestampSeconds()));
        Logger.recordOutput(
            "AprilTagVision/" + camera.getName() + "/TrigEstimate/DEBUG/TagXYRotation",
            closestTargetPose.get().getRotation().getZ());
        Logger.recordOutput(
            "AprilTagVision/" + camera.getName() + "/TrigEstimate/DEBUG/bestTarget",
            FieldConstants.aprilTagLayout.getTagPose(result.getBestTarget().fiducialId).get());
      }

      // Add pose observation
      if (result.multitagResult.isPresent()) { // Multitag result
        var multitagResult = result.multitagResult.get();

        // Calculate robot pose
        Transform3d fieldToCamera = multitagResult.estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(cameraDisplacement.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        // Calculate average tag distance
        double totalTagDistance = 0.0;
        for (var target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }

        // Add tag IDs
        tagIds.addAll(multitagResult.fiducialIDsUsed);

        // Add observation
        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(), // Timestamp
                robotPose, // 3D pose estimate
                multitagResult.estimatedPose.ambiguity, // Ambiguity
                multitagResult.fiducialIDsUsed.size(), // Tag count
                totalTagDistance / result.targets.size() // Average tag distance
                ));

      } else if (!result.targets.isEmpty()) { // Single tag result
        // TODO Photonvision solution --leaving it in for now, but will this waste processing power?
        var target = result.targets.get(0);

        // Calculate robot pose
        var tagPose = FieldConstants.aprilTagLayout.getTagPose(target.fiducialId);
        if (tagPose.isPresent()) {
          Transform3d fieldToTarget =
              new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
          Transform3d cameraToTarget = target.bestCameraToTarget;
          Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
          Transform3d fieldToRobot = fieldToCamera.plus(cameraDisplacement.inverse());
          Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

          // Add tag ID
          tagIds.add((short) target.fiducialId);

          // Add observation
          poseObservations.add(
              new PoseObservation(
                  result.getTimestampSeconds(), // Timestamp
                  robotPose, // 3D pose estimate
                  target.poseAmbiguity, // Ambiguity
                  1, // Tag count
                  cameraToTarget.getTranslation().getNorm() // Average tag distance
                  )); // Observation type
        }
      }
    }

    // Save pose observations to inputs object
    inputs.photonPoseObservations = new PoseObservation[poseObservations.size()];
    Logger.recordOutput("size", poseObservations.size());
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.photonPoseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }

    inputs.trigTargetObservations =
        trigObservations.toArray(new TrigTargetObservation[trigObservations.size()]);
  }
}
