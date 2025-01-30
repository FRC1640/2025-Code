package frc.robot.subsystems.intakeoutake;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.RobotConstants.IntakeConstants;
import frc.robot.constants.SparkConstants;
import frc.robot.util.spark.SparkConfigurer;

public class IntakeOutakeIOSparkMax implements IntakeOutakeIO {
  private final SparkMax intakeSpark;

  public IntakeOutakeIOSparkMax() {
    intakeSpark =
        SparkConfigurer.configSparkMax(
            SparkConstants.getGantryDefaultSparkMax(IntakeConstants.intakeSparkID));
  }

  @Override
  public void updateInputs(IntakeOutakeIOInputs inputs) {
    inputs.tempCelcius = intakeSpark.getMotorTemperature();
    inputs.appliedVoltage = intakeSpark.getAppliedOutput() * RobotController.getBatteryVoltage();
  }

  @Override
  public void setIntakeVoltage(double voltage) {
    intakeSpark.setVoltage(clampVoltage(voltage));
  }
}
