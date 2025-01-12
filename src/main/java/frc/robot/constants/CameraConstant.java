package frc.robot.constants;

import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.simulation.SimCameraProperties;

public class CameraConstant {
  public final SimCameraProperties cameraProperties;
  public final Transform3d transform;
  public final double standardDevConstant;
  public final String name;

  public CameraConstant(
      SimCameraProperties cameraProperties,
      Transform3d transform,
      double standardDevConstant,
      String name) {
    this.cameraProperties = cameraProperties;
    this.transform = transform;
    this.standardDevConstant = standardDevConstant;
    this.name = name;
  }
}
