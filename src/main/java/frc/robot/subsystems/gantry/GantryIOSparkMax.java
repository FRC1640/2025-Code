package frc.robot.subsystems.gantry;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.RobotConstants.GantryConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.constants.SparkConstants;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.tools.MotorLim;

public class GantryIOSparkMax implements GantryIO {
  private final SparkMax gantrySpark;
  private final RelativeEncoder gantryEncoder;
  private final PIDController gantryPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.gantryPID);

  public GantryIOSparkMax() {
    gantrySpark =
        SparkConfigurer.configSparkMax(
            SparkConstants.getGantryDefaultSparkMax(GantryConstants.gantrySparkID));
    gantryEncoder = gantrySpark.getEncoder();
  }

  @Override
  public void updateInputs(GantryIOInputs inputs) {
    inputs.currentAmps = gantrySpark.getOutputCurrent();
    inputs.tempCelcius = gantrySpark.getMotorTemperature();
    inputs.appliedVoltage = gantrySpark.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.encoderPosition = gantryEncoder.getPosition();
  }

  @Override
  public void setGantryVoltage(double voltage, GantryIOInputs inputs) {
    gantrySpark.setVoltage(
        MotorLim.applyLimits(
            inputs.encoderPosition, MotorLim.clampVoltage(voltage), GantryConstants.gantryLimits));
  }

  @Override
  public void setGantryPosition(double position, GantryIOInputs inputs) {
    setGantryVoltage(gantryPID.calculate(inputs.encoderPosition, position), inputs);
  }
}
