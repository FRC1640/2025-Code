package frc.robot.subsystems.gantry;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.RobotConstants.GantryConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.constants.SparkConstants;

public class GantryIOSparkMax implements GantryIO {
  private final SparkMax carriageSpark;
  private final RelativeEncoder carriageEncoder;
  private final PIDController gantryPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.gantryPID);

  public GantryIOSparkMax() {
    carriageSpark = SparkConstants.getGantryDefaultSparkMax(GantryConstants.gantrySparkID);
    carriageEncoder = carriageSpark.getEncoder();
  }

  @Override
  public void updateInputs(GantryIOInputs inputs) {
    inputs.currentAmps = carriageSpark.getOutputCurrent();
    inputs.tempCelcius = carriageSpark.getMotorTemperature();
    inputs.appliedVoltage = carriageSpark.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.encoderPosition = carriageEncoder.getPosition();
  }

  @Override
  public void setGantryVoltage(double voltage, GantryIOInputs inputs) {
    voltage = clampVoltage(voltage);
    voltage = applyLimits(inputs.encoderPosition, voltage);
    carriageSpark.setVoltage(voltage);
  }

  @Override
  public void setCarriagePosition(double position, GantryIOInputs inputs) {
    setGantryVoltage(gantryPID.calculate(inputs.encoderPosition, position), inputs);
  }
}
