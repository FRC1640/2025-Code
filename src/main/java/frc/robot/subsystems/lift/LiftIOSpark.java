package frc.robot.subsystems.lift;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import frc.robot.constants.RobotConstants.LiftConstants;
import frc.robot.constants.SparkConstants;
import frc.robot.util.spark.SparkConfigurer;

public class LiftIOSpark implements LiftIO {
  RelativeEncoder motorEncoder1;
  RelativeEncoder motorEncoder2;
  SparkMax motor1;
  SparkMax motor2;

  public LiftIOSpark() {
    motor1 =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(LiftConstants.liftMotor1ID, false));
    motor2 =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(LiftConstants.liftMotor2ID, false));
    motorEncoder1 = motor1.getEncoder();
    motorEncoder2 = motor2.getEncoder();
  }

  @Override
  public void updateInputs(LiftIOInputs inputs) {
    inputs.liftmotor1Position = motorEncoder1.getPosition();
    inputs.liftmotor2Position = motorEncoder2.getPosition();
    inputs.liftmotor1Velocity = motorEncoder1.getVelocity();
    inputs.liftmotor2Velocity = motorEncoder2.getVelocity();
    inputs.liftmotor1Current = motor1.getOutputCurrent();
    inputs.liftmotor2Current = motor2.getOutputCurrent();
    inputs.liftmotor1Voltage = motor1.getAppliedOutput();
    inputs.liftmotor2Voltage = motor2.getAppliedOutput();
  }
}
