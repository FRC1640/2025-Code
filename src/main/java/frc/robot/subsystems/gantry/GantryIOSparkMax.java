package frc.robot.subsystems.gantry;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.RobotConstants.GantryConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.constants.SparkConstants;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.tools.MotorLim;

public class GantryIOSparkMax implements GantryIO {
  private final SparkMax gantrySpark;
  private final RelativeEncoder gantryEncoder;
  private final SparkLimitSwitch gantryLimitSwitch;
  private final SimpleMotorFeedforward ff =
      RobotPIDConstants.constructFFSimpleMotor(RobotPIDConstants.gantryFF);
  private final PIDController gantryPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.gantryPID);
  private final PIDController gantryVelocityPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.gantryVelocityPID);

  public GantryIOSparkMax() {
    gantrySpark =
        SparkConfigurer.configSparkMax(
            SparkConstants.getGantryDefaultSparkMax(GantryConstants.gantrySparkID));
    gantryEncoder = gantrySpark.getEncoder();
    gantryLimitSwitch = gantrySpark.getForwardLimitSwitch();
  }

  @Override
  public void updateInputs(GantryIOInputs inputs) {
    inputs.currentAmps = gantrySpark.getOutputCurrent();
    inputs.tempCelcius = gantrySpark.getMotorTemperature();
    inputs.appliedVoltage = gantrySpark.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.encoderPosition =
        gantryEncoder.getPosition()
            / GantryConstants.gantryGearRatio
            * GantryConstants.pulleyRadius
            * 2
            * Math.PI;
    inputs.isLimitSwitchPressed = gantryLimitSwitch.isPressed();
    inputs.encoderVelocity =
        gantryEncoder.getVelocity()
            / 60
            * 2
            * Math.PI
            / GantryConstants.gantryGearRatio
            * GantryConstants.pulleyRadius;
  }

  public void setGantryVoltage(
      double voltage,
      GantryIOInputs inputs,
      boolean limit) { // right limit is boolean condition for limitswitch
    gantrySpark.setVoltage(
        MotorLim.applyLimits(
            inputs.encoderPosition,
            MotorLim.clampVoltage(voltage),
            GantryConstants.gantryLimits.low,
            limit ? GantryConstants.gantryLimits.high : null));
  }

  public void setGantryPosition(double position, GantryIOInputs inputs, boolean limit) {
    setGantryVoltage(
        gantryPID.calculate(inputs.encoderPosition, position), inputs, GantrySubsystem.limit);
  }

  @Override
  public void setGantryVelocity(double velocity, GantryIOInputs inputs, boolean limit) {
    setGantryVoltage(
        ff.calculate(velocity) + gantryVelocityPID.calculate(inputs.encoderVelocity, velocity),
        inputs,
        limit);
  }

  @Override
  public void resetEncoder() {
    gantryEncoder.setPosition(0);
  }
}
