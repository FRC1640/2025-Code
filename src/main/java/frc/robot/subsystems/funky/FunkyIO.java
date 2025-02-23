package frc.robot.subsystems.funky;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.constants.SparkConstants;
import frc.robot.util.PID.SparkPIDController;
import frc.robot.util.spark.SparkConfigurer;

public class FunkyIO {
  private final SparkMax sparkMax;
  SparkPIDController sparkPIDController;
  RelativeEncoder encoder;

  public FunkyIO() {
    sparkMax =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(13, false)
                .applyPIDConfig(RobotPIDConstants.pidConstantSpark));
    sparkPIDController = new SparkPIDController(sparkMax.getClosedLoopController(), "FunkySpark");
    encoder = sparkMax.getEncoder();
    setVelocity(0);
  }

  public void setPos(double pos) {
    sparkPIDController.setReference(pos, ControlType.kMAXMotionPositionControl);
  }

  public void setVelocity(double velocity) {
    sparkPIDController.setReference(velocity, ControlType.kMAXMotionVelocityControl);
  }
}
