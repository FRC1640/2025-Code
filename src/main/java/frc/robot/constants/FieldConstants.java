package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import org.photonvision.simulation.VisionSystemSim;

public class FieldConstants {
  public static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  public static final VisionSystemSim fieldSim = new VisionSystemSim("sim field");

  static {
    fieldSim.addAprilTags(aprilTagLayout);
  }

  public static final double height = 8.21;
  public static final double width = 16.54;
}
