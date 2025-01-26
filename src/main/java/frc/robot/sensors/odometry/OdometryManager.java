package frc.robot.sensors.odometry;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.util.periodic.PeriodicBase;

public class OdometryManager extends PeriodicBase {
  private SwerveDriveKinematics kinematics;
  private Supplier<Rotation2d> headingSupplier;
  private Supplier<SwerveModulePosition[]> positionsSupplier;

  private Map<SwerveDrivePoseEstimator, Consumer<SwerveDrivePoseEstimator>> updatersMap = new HashMap<>();
  private Map<SwerveDrivePoseEstimator, String> namesMap = new HashMap<>();

  public OdometryManager(SwerveDriveKinematics kinematics, Supplier<Rotation2d> headingSupplier, Supplier<SwerveModulePosition[]> positionsSupplier) {
    this.kinematics = kinematics;
    this.headingSupplier = headingSupplier;
    this.positionsSupplier = positionsSupplier;
  }

  public void register(SwerveDrivePoseEstimator estimator, String name, Consumer<SwerveDrivePoseEstimator> updater) {
    if (estimator == null) {
      throw new IllegalArgumentException("Cannot register null pose estimator");
    }
    if (updater == null) {
      throw new IllegalArgumentException("Cannot register null updater");
    }
    if (name == null) {
      throw new IllegalArgumentException("Cannot register pose estimator with null name");
    }

    if (updatersMap.containsKey(estimator)) {
      throw new IllegalStateException("Cannot register pose estimator that is already registered");
    }

    updatersMap.put(estimator, updater);
    namesMap.put(estimator, name);
  }

  public void modify(SwerveDrivePoseEstimator estimator, Consumer<SwerveDrivePoseEstimator> updater) {
    if (!updatersMap.containsKey(estimator)) {
      throw new IllegalArgumentException("Cannot modify pose estimator that is not registered");
    }
    if (updater == null) {
      throw new IllegalArgumentException("Cannot register null updater");
    }

    updatersMap.put(estimator, updater);
  }

  public boolean deregister(SwerveDrivePoseEstimator estimator) {
    if (!updatersMap.containsKey(estimator)) {
      return false;
    }

    updatersMap.remove(estimator);
    namesMap.remove(estimator);
    return true;
  }

  public SwerveDrivePoseEstimator branch(SwerveDrivePoseEstimator from, String name, Consumer<SwerveDrivePoseEstimator> updater) {
    if (!updatersMap.containsKey(from)) {
      throw new IllegalArgumentException("Cannot branch from pose estimator that is not registered");
    }
    if (updater == null) {
      throw new IllegalArgumentException("Cannot branch from pose estimator with a null updater");
    }
    if (name == null) {
      throw new IllegalArgumentException("Cannot branch from pose estimator with null name");
    }

    SwerveDrivePoseEstimator branchedEstimator = new SwerveDrivePoseEstimator(kinematics, headingSupplier.get(), positionsSupplier.get(), from.getEstimatedPosition());

    updatersMap.put(branchedEstimator, updater);
    namesMap.put(branchedEstimator, name);

    return branchedEstimator;
  }

  public SwerveDrivePoseEstimator branch(SwerveDrivePoseEstimator from, String name) {
    if (!updatersMap.containsKey(from)) {
      throw new IllegalArgumentException("Cannot branch from pose estimator that is not registered");
    }

    var sourceUpdater = updatersMap.get(from);
    return branch(from, name, sourceUpdater);
  }

  @Override
  public void periodic() {
    for (SwerveDrivePoseEstimator estimator : updatersMap.keySet()) {
      String name = namesMap.get(estimator);
      var updater = updatersMap.get(estimator);

      if (estimator == null || updater == null || name == null) {
        continue;
      }

      updater.accept(estimator);
      Logger.recordOutput("Drive/Odometry/" + name, estimator.getEstimatedPosition());
    }
  }
}
