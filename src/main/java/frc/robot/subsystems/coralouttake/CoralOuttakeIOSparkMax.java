package frc.robot.subsystems.coralouttake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.RobotConstants.CoralOuttakeConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.constants.SparkConstants;
import frc.robot.util.misc.MotorLim;
import frc.robot.util.spark.SparkConfigurer;

public class CoralOuttakeIOSparkMax implements CoralOuttakeIO {
  private final SparkMax intakeSpark;
  // private final DigitalInput coralDetector;
  private final DigitalInput hasCoralDetector;
  private final SimpleMotorFeedforward ff =
      RobotPIDConstants.constructFFSimpleMotor(RobotPIDConstants.coralFF, "coralFF");
  private final PIDController coralVelocityPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.coralVelocityPID, "coralVelocityPID");

  public CoralOuttakeIOSparkMax() {
    intakeSpark =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMaxIntake(
                CoralOuttakeConstants.intakeSparkID, false, IdleMode.kBrake));
    // coralDetector = new DigitalInput(CoralOuttakeConstants.coralDetectorChannel);
    hasCoralDetector = new DigitalInput(CoralOuttakeConstants.hasCoralDetectorChannel);
  }

  @Override
  public void updateInputs(CoralOuttakeIOInputs inputs) {
    inputs.tempCelcius = intakeSpark.getMotorTemperature();
    inputs.appliedVoltage = intakeSpark.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.coralDetectedHigh = intakeSpark.getReverseLimitSwitch().isPressed();
    inputs.outtakeVelocity = intakeSpark.getEncoder().getVelocity();
    inputs.hasCoral = !hasCoralDetector.get();
  }

  @Override
  public void setIntakeVoltage(double voltage) {
    intakeSpark.setVoltage(MotorLim.clampVoltage(voltage));
  }

  @Override
  public void setIntakeVelocity(double velocity, CoralOuttakeIOInputs inputs) {
    setIntakeVoltage(
        ff.calculate(velocity) + coralVelocityPID.calculate(inputs.outtakeVelocity, velocity));
  }
}
