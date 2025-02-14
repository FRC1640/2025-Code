package frc.robot.subsystems.funky;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.constants.SparkConstants;
import frc.robot.util.spark.SparkConfigurer;

public class FunkyIO {
  private final SparkMax sparkMax;
  SparkClosedLoopController sparkClosedLoopController;
  RelativeEncoder encoder;

  public FunkyIO() {
    sparkMax =
        SparkConfigurer.configSparkMax(
            SparkConstants.getGantryDefaultSparkMax(13)
                .applyPIDConfig(RobotPIDConstants.pidConstantSpark));
    sparkClosedLoopController = sparkMax.getClosedLoopController();
    encoder = sparkMax.getEncoder();
    setVelocity(0);
  }

  public void setPos(double pos) {
    sparkClosedLoopController.setReference(pos, ControlType.kMAXMotionPositionControl);
  }

  public void setVelocity(double velocity) {
    sparkClosedLoopController.setReference(velocity, ControlType.kMAXMotionVelocityControl);
  }
}
