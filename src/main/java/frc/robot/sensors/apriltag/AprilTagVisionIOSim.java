package frc.robot.sensors.apriltag;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.FieldConstants;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagVisionIOSim implements AprilTagVisionIO {
  private final PhotonCamera camera;
  private final PhotonCameraSim cameraSim;
  private final Transform3d cameraDisplacement;

  public AprilTagVisionIOSim(
      String name, SimCameraProperties properties, Transform3d cameraDisplacement) {
    this.camera = new PhotonCamera(name);
    this.cameraSim = new PhotonCameraSim(camera, properties);
    this.cameraDisplacement = cameraDisplacement;
    FieldConstants.fieldSim.addCamera(cameraSim, cameraDisplacement);
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    inputs.connected = camera.isConnected();
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
      if (result.hasTargets()) {
        PhotonTrackedTarget target = result.getBestTarget();
        inputs.latestTargetObservation =
            new AprilTagObservation(
                new Rotation2d(target.getYaw()), new Rotation2d(target.getPitch()));
      } else {
        inputs.latestTargetObservation =
            new AprilTagObservation(new Rotation2d(), new Rotation2d());
      }
      if (result.multitagResult.isPresent()) {
        MultiTargetPNPResult multitag = result.getMultiTagResult().get();
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
      } else if (!result.targets
          .isEmpty()) { // if we don't have a multitatargg result, check if we even have a target
        PhotonTrackedTarget target = result.targets.get(0);
        Transform3d cameraTransform = target.getBestCameraToTarget();
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
                cameraTransform.getTranslation().getNorm()));
      }
    }
    inputs.poseObservations =
        poseObservations.toArray(new PoseObservation[poseObservations.size()]);
    int index = 0;
    for (Short id : tagIds) {
      inputs.tagIds[index] = id;
      index++;
    }
  }
}
