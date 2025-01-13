package frc.robot.sensors.apriltag;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.CameraConstant;
import frc.robot.constants.FieldConstants;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;

public class AprilTagVisionIOSim extends AprilTagVisionIOPhotonvision {
  private final PhotonCameraSim cameraSim;
  private final Supplier<Pose3d> getPose;

  public AprilTagVisionIOSim(
      String name, SimCameraProperties properties, Transform3d cameraDisplacement) {
    this.camera = new PhotonCamera(name);
    this.cameraSim = new PhotonCameraSim(camera, properties);
    this.cameraDisplacement = cameraDisplacement;
    FieldConstants.fieldSim.addCamera(cameraSim, cameraDisplacement);
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    FieldConstants.fieldSim.update(getPose.get());
    super.updateInputs(inputs);
  }
}
