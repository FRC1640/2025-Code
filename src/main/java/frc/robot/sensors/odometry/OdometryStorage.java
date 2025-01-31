package frc.robot.sensors.odometry;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.sensors.apriltag.AprilTagVision;
import frc.robot.sensors.odometry.RobotOdometry.VisionUpdateMode;

public class OdometryStorage {
  public SwerveDrivePoseEstimator estimator;
  public Rotation2d rawGyroRotation = new Rotation2d();
  private AprilTagVision[] visions;
  private VisionUpdateMode updateMode;
  private String name;

  public String getName() {
    return name;
  }

  public OdometryStorage(
      String name,
      SwerveDrivePoseEstimator estimator,
      AprilTagVision[] visions,
      VisionUpdateMode updateMode) {
    this.estimator = estimator;
    this.visions = visions;
    this.updateMode = updateMode;
    this.name = name;
  }

  public AprilTagVision[] getVisions() {
    return visions;
  }

  public VisionUpdateMode getUpdateMode() {
    return updateMode;
  }

  public SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  @Override
  public int hashCode() {
    return estimator.hashCode();
  }
}
