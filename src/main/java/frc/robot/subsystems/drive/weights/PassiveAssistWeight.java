package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.RobotConstants.AutoAlignConfig;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class PassiveAssistWeight implements DriveWeight {
  private DriveWeight inner;
  private DoubleSupplier getDistance;
  private BooleanSupplier assistOverride;

  public PassiveAssistWeight(
      DriveWeight inner, DoubleSupplier distance, BooleanSupplier assistOverride) {
    this.inner = inner;
    this.getDistance = distance;
    this.assistOverride = assistOverride;
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    return inner.getSpeeds();
  }

  @Override
  public double getWeight() {
    double weight;
    if (getDistance.getAsDouble() > AutoAlignConfig.passiveDistThresh
        || assistOverride.getAsBoolean()) {
      weight = 0;
    } else {
      weight =
          AutoAlignConfig.passiveWeightBase * (1 - 1 / (1 + Math.exp(getDistance.getAsDouble())));
    }
    Logger.recordOutput("LocalTagAlign/persistentWeight", weight);
    return weight;
  }
}
