package frc.robot.subsystems.funky;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import frc.robot.constants.SparkConstants;
import frc.robot.util.spark.SparkConfigurer;

public class FunkyIO {
  private final SparkMax sparkMax;
  SparkClosedLoopController sparkClosedLoopController;

  public FunkyIO() {
    sparkMax = SparkConfigurer.configSparkMax(SparkConstants.getGantryDefaultSparkMax(13));
    sparkClosedLoopController = sparkMax.getClosedLoopController();
  }

  public void setPos(double pos) {
    // sparkClosedLoopController.setReference(pos, ControlType.kMAXMotionPositionControl);
  }

  public void setVelocity(double velocity) {
    // sparkClosedLoopController.setReference(velocity, ControlType.kMAXMotionVelocityControl);
  }
}
