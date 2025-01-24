package frc.robot.subsystems.lift;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.SparkConstants;

public class LiftIOSparkMax implements LiftIO {
  private final SparkMax liftSpark;
  private final RelativeEncoder liftEncoder;

  public LiftIOSparkMax() {
    liftSpark = SparkConstants.getDefaultSparkMax(-1); // update when we have id
    liftEncoder = liftSpark.getEncoder();
  }

  @Override
  public void updateInputs(LiftIOInputs inputs) {
    inputs.currentAmps = liftSpark.getOutputCurrent();
    inputs.tempCelcius = liftSpark.getMotorTemperature();
    inputs.appliedVoltage = liftSpark.getAppliedOutput() * RobotController.getBatteryVoltage();
  }

  public void setLiftVoltage(double voltage) {
    liftSpark.setVoltage(voltage);
  }
}
