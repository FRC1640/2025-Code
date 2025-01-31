package frc.robot.subsystems.coralouttake;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.RobotConstants.IntakeConstants;
import frc.robot.constants.SparkConstants;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.tools.MotorLim;

public class CoralOuttakeIOSparkMax implements CoralOuttakeIO {
  private final SparkMax intakeSpark;

  public CoralOuttakeIOSparkMax() {
    intakeSpark =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(IntakeConstants.intakeSparkID, false));
  }

  @Override
  public void updateInputs(CoralOuttakeIOInputs inputs) {
    inputs.tempCelcius = intakeSpark.getMotorTemperature();
    inputs.appliedVoltage = intakeSpark.getAppliedOutput() * RobotController.getBatteryVoltage();
  }

  @Override
  public void setIntakeVoltage(double voltage) {

    intakeSpark.setVoltage(MotorLim.clampVoltage(voltage));
  }
}
