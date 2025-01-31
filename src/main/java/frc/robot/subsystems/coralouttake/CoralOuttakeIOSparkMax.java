package frc.robot.subsystems.coralouttake;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.RobotConstants.CoralOuttakeConstants;
import frc.robot.constants.SparkConstants;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.tools.MotorLim;

public class CoralOuttakeIOSparkMax implements CoralOuttakeIO {
  private final SparkMax intakeSpark;
  private DigitalInput coralDetector;

  public CoralOuttakeIOSparkMax() {
    intakeSpark =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(CoralOuttakeConstants.intakeSparkID, false));
    coralDetector = new DigitalInput(CoralOuttakeConstants.coralDetectorChannel);
  }

  @Override
  public void updateInputs(CoralOuttakeIOInputs inputs) {
    inputs.tempCelcius = intakeSpark.getMotorTemperature();
    inputs.appliedVoltage = intakeSpark.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.coralDetected = !coralDetector.get();
    if (CoralOuttakeConstants.encoderOnIntakeMotor) {
      inputs.outtakeVelocity = intakeSpark.getEncoder().getVelocity();
    }
  }

  @Override
  public void setIntakeVoltage(double voltage) {

    intakeSpark.setVoltage(MotorLim.clampVoltage(voltage));
  }
}
