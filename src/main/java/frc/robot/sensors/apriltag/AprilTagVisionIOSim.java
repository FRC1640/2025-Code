package frc.robot.sensors.apriltag;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.constants.CameraConstant;
import frc.robot.constants.FieldConstants;

import java.util.function.IntSupplier;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;

public class AprilTagVisionIOSim extends AprilTagVisionIOPhotonvision {
  private final PhotonCameraSim cameraSim;
  private final Supplier<Pose3d> getPose;

  public AprilTagVisionIOSim(CameraConstant constant, Supplier<Pose3d> getPose) {
    super(constant);
    this.getPose = getPose;
    this.cameraSim = new PhotonCameraSim(camera, constant.cameraProperties);
    FieldConstants.visionSim.addAprilTags(FieldConstants.aprilTagLayout);
    FieldConstants.visionSim.addCamera(cameraSim, cameraDisplacement);
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    FieldConstants.visionSim.update(getPose.get());
    super.updateInputs(inputs);
  }
}
