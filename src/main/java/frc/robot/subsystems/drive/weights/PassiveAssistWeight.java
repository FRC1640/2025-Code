package frc.robot.subsystems.drive.weights;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.RobotConstants.AutoAlignConfig;

public class PassiveAssistWeight implements DriveWeight {

  private DriveWeight inner;
  private BooleanSupplier assistOverride;

  public PassiveAssistWeight(DriveWeight inner, BooleanSupplier assistOverride) {
    this.inner = inner;
    this.assistOverride = assistOverride;
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    return inner.getSpeeds();
  }

  @Override
  public double getWeight() {
    double weight;
    Pose2d target = getTarget();
    Pose2d robot = getRobotPose.get();
    double distance = robot.getTranslation().getDistance(target.getTranslation());
    if (distance > AutoAlignConfig.passiveDistThresh
        || assistOverride.getAsBoolean()) {
      weight = 0;
    } else {
      weight = AutoAlignConfig.passiveWeightBase * (1 - 1 / (1 + Math.exp(distance)));
    }
    Logger.recordOutput("LocalTagAlign/persistentWeight", weight);
    return weight;
  }
}
