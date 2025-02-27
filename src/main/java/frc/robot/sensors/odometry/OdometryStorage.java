package frc.robot.sensors.odometry;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.sensors.apriltag.AprilTagVision;
import frc.robot.sensors.odometry.RobotOdometry.VisionUpdateMode;
import java.util.NoSuchElementException;
import java.util.Optional;

public class OdometryStorage {
  public SwerveDrivePoseEstimator estimator;
  public Rotation2d rawGyroRotation = new Rotation2d();
  private AprilTagVision[] visions;
  private VisionUpdateMode updateMode;
  private String name;
  private Optional<OdometryStorage> trustedRotation;
  private final double gyroBufferSizeSec = 2.0;
  private TimeInterpolatableBuffer<Rotation2d> gyroBuffer =
      TimeInterpolatableBuffer.createBuffer(
          (a, b, c) -> {
            double aRadians = a.getRadians();
            double delta = (b.getRadians() % (2 * Math.PI)) - aRadians;
            return new Rotation2d((delta * c) + aRadians);
          },
          gyroBufferSizeSec);

  public void setTrustedRotation(OdometryStorage trustedRotation) {
    this.trustedRotation = Optional.of(trustedRotation);
  }

  public Optional<OdometryStorage> getTrustedRotation() {
    return trustedRotation;
  }

  public void addGyroSample(Rotation2d sample, double timestamp) {
    gyroBuffer.addSample(timestamp, sample);
  }

  public Optional<Rotation2d> getGyroAtTimestamp(double timestamp) {
    try {
      if (gyroBuffer.getInternalBuffer().lastKey() - gyroBufferSizeSec > timestamp) {
        return Optional.empty();
      }
    } catch (NoSuchElementException e) {
      return Optional.empty();
    }
    return gyroBuffer.getSample(timestamp);
  }

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
    trustedRotation = Optional.empty();
  }

  public OdometryStorage(
      String name,
      SwerveDrivePoseEstimator estimator,
      AprilTagVision[] visions,
      VisionUpdateMode updateMode,
      OdometryStorage trustedRotation) {
    this.estimator = estimator;
    this.visions = visions;
    this.updateMode = updateMode;
    this.name = name;
    this.trustedRotation = Optional.of(trustedRotation);
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
