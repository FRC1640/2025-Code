package frc.robot.sensors.odometry;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class OdometryStorage {
  public SwerveDrivePoseEstimator estimator;
  public Rotation2d rawGyroRotation = new Rotation2d();

  public OdometryStorage(SwerveDrivePoseEstimator estimator) {
    this.estimator = estimator;
  }

  public SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
}
