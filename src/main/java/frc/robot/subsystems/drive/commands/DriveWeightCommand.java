package frc.robot.subsystems.drive.commands;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.subsystems.drive.weights.DriveWeight;

public class DriveWeightCommand {
  static ArrayList<DriveWeight> persistentWeights = new ArrayList<>();

  static ArrayList<DriveWeight> weights = new ArrayList<>();

  private static DriveSpeedFilterer filter = new DriveSpeedFilterer();

  public static Command create(
      DriveCommandFactory driveCommandFactory) {
    filter.configure();
    Command c = driveCommandFactory.runVelocityCommand(() -> getAllSpeeds());
    return c;
  }

  public static void addWeight(DriveWeight weight) {
    if (!weights.contains(weight)) {
      weights.add(weight);
    }
  }

  public static void removeWeight(DriveWeight weight) {
    if (weights.contains(weight)) {
      weights.remove(weight);
    }
  }

  public static void addPersistentWeight(DriveWeight weight) {
    if (!persistentWeights.contains(weight)) {
      persistentWeights.add(weight);
    }
  }

  public static void removePersistentWeight(DriveWeight weight) {
    if (persistentWeights.contains(weight)) {
      persistentWeights.remove(weight);
    }
  }

  public static void removeAllWeights() {
    weights.clear();
  }

  private static ChassisSpeeds getAllSpeeds() {
    ChassisSpeeds speeds = new ChassisSpeeds();
    // remove weights with a true cancel condition
    Iterator<DriveWeight> iterator = weights.iterator();
    while (iterator.hasNext()) {
      DriveWeight weight = iterator.next();
      if (weight.cancelCondition()) {
        iterator.remove();
      }
    }

    double weightSum = 0;
    // iterate over remaining weights and add speeds
    for (DriveWeight driveWeight : weights) {
      speeds = speeds.plus(driveWeight.getSpeeds().times(driveWeight.getWeight()));
      weightSum += driveWeight.getWeight();
    }
    for (DriveWeight driveWeight : persistentWeights) {
      speeds = speeds.plus(driveWeight.getSpeeds().times(driveWeight.getWeight()));
      weightSum += driveWeight.getWeight();
    }
    speeds.div(weightSum);
    return decreaseSpeeds(speeds);
  }

  public static int getWeightsSize() {
    return weights.size() + persistentWeights.size();
  }

  public static ChassisSpeeds decreaseSpeeds(ChassisSpeeds speeds) {
    double max =
        Math.max(
            Math.hypot(
                speeds.vxMetersPerSecond / DriveConstants.maxSpeed,
                speeds.vyMetersPerSecond / DriveConstants.maxSpeed),
            speeds.omegaRadiansPerSecond / DriveConstants.maxOmega);
    if (max > 1) {
      return speeds.times(1 / max);
      // System.out.println(speeds);
    }

    return speeds;
  }

  public static Trigger createWeightTrigger(DriveWeight weight, BooleanSupplier condition) {
    new Trigger(() -> weight.cancelCondition())
        .onTrue(new InstantCommand(() -> removeWeight(weight)));
    return new Trigger(condition)
        .onTrue(
            new InstantCommand(
                () -> {
                  addWeight(weight);
                  weight.onStart();
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  removeWeight(weight);
                  weight.onFinish();
                }));
  }

  /* public static void createWeightTrigger(DriveWeight weight, BooleanSupplier condition, BooleanSupplier startRequirement) {
    new Trigger(() -> weight.cancelCondition())
        .onTrue(new InstantCommand(() -> removeWeight(weight)));
    new Trigger(() -> condition.getAsBoolean() && startRequirement.getAsBoolean())
        .onTrue(new InstantCommand(() -> {addWeight(weight); weight.onStart();}));
    new Trigger(condition)
        .onFalse(new InstantCommand(() -> removeWeight(weight)));
  } */

  public static boolean checkWeight(DriveWeight weight) {
    return weights.contains(weight) || persistentWeights.contains(weight);
  }

  private static class DriveSpeedFilterer {
    private BooleanSupplier hasCoral;
    private DoubleSupplier getCoralStationDistance;
    private Supplier<Pose2d> getRobotPose;

    private DriveSpeedFilterer() {}

    public void configure(BooleanSupplier hasCoral, DoubleSupplier getCoralStationDistance, Supplier<Pose2d> getRobotPose) {
      this.hasCoral = hasCoral;
      this.getCoralStationDistance = getCoralStationDistance;
      this.getRobotPose = getRobotPose;
    }

    public ChassisSpeeds restrictSpeeds(ChassisSpeeds speeds) {
      Translation2d velocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
      if (velocity.getAngle())
    }
  }
}