package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants.AutoAlignConfig;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.helpers.AutoAlignHelper;
import frc.robot.util.misc.AllianceManager;
import frc.robot.util.misc.DistanceManager;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class PassiveStationAlignWeight implements DriveWeight {
  private static final double ASSIST_WEIGHT_BASE = 0.6;

  private Supplier<Pose2d> getRobotPose;
  private AutoAlignHelper autoAlignHelper;
  private Gyro gyro;
  private DriveSubsystem driveSubsystem;
  private BooleanSupplier hasCoral;
  private Function<Pose2d, Pose2d> poseFunction;

  public PassiveStationAlignWeight(
      Supplier<Pose2d> getRobotPose,
      Gyro gyro,
      DriveSubsystem driveSubsystem,
      BooleanSupplier hasCoral,
      Function<Pose2d, Pose2d> poseFunction) {
    this.getRobotPose = getRobotPose;
    this.autoAlignHelper = new AutoAlignHelper();
    this.gyro = gyro;
    this.driveSubsystem = driveSubsystem;
    this.hasCoral = hasCoral;
    this.poseFunction = poseFunction;
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    Pose2d target = getTarget();
    Pose2d robot = getRobotPose.get();
    return autoAlignHelper.getPassiveStationSpeedsXY(robot, target, gyro);
  }

  private Pose2d getTarget() {
    Pose2d target = poseFunction == null
        ? DistanceManager.getNearestPosition(
            getRobotPose.get(),
            AllianceManager.chooseFromAlliance(
                FieldConstants.coralStationPosBlue, FieldConstants.coralStationPosRed))
        : DistanceManager.getNearestPosition(
            getRobotPose.get(),
            AllianceManager.chooseFromAlliance(
                FieldConstants.coralStationPosBlue, FieldConstants.coralStationPosRed),
            poseFunction);
    Logger.recordOutput("PassiveStationAlign/target", target);
    return target;
  }

  @Override
  public double getWeight() {
    double weight;
    Pose2d target = getTarget();
    Pose2d robot = getRobotPose.get();
    double distance = robot.getTranslation().getDistance(target.getTranslation());
    if (distance > AutoAlignConfig.stationAssistDistThresh
        || hasCoral.getAsBoolean()
        || autoAlignHelper.isStrafing(
            target.getRotation(),
            driveSubsystem.getChassisSpeeds(),
            Math.PI / 2)) {
      weight = 0;
    } else {
      weight = ASSIST_WEIGHT_BASE * (1 - 1 / (1 + Math.exp(distance)));
    }
    Logger.recordOutput("LocalTagAlign/persistentWeight", weight);
    return weight;
  }
}
