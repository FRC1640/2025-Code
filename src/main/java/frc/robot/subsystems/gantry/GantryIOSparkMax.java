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
  private double velocitySetpoint = 0;
  private final SparkMax gantrySpark;
  private final RelativeEncoder gantryEncoder;
  private final SparkLimitSwitch gantryLimitSwitch;
  private final SimpleMotorFeedforward ff =
      RobotPIDConstants.constructFFSimpleMotor(RobotPIDConstants.gantryFF);
  private final PIDController gantryPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.gantryPID);
  private final PIDController gantryVelocityPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.gantryVelocityPID);
  private boolean limits = false;

  public GantryIOSparkMax() {
    gantrySpark =
        SparkConfigurer.configSparkMax(
            SparkConstants.getGantryDefaultSparkMax(GantryConstants.gantrySparkID)
                .applyPIDConfig(RobotPIDConstants.pidConstantSpark));
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
      double voltage, GantryIOInputs inputs) { // right limit is boolean condition for limitswitch
    gantrySpark.setVoltage(
        MotorLim.applyLimits(
            inputs.encoderPosition,
            MotorLim.clampVoltage(voltage),
            GantryConstants.gantryLimits.low,
            limits ? GantryConstants.gantryLimits.high : null));
  }

  public void setGantryPosition(double position, GantryIOInputs inputs) {
    setGantryVoltage(gantryPID.calculate(inputs.encoderPosition, position), inputs);
  }

  @Override
  public void setGantryVelocity(double velocity, GantryIOInputs inputs) {
    setGantryVoltage(
        ff.calculate(velocity) + gantryVelocityPID.calculate(inputs.encoderVelocity, velocity),
        inputs);
    velocitySetpoint = velocity;
  }

  @Override
  public void resetEncoder() {
    gantryEncoder.setPosition(0);
  }

  @Override
  public double velocitySetpoint() {
    return velocitySetpoint;
  }

  @Override
  public void homedLimit() {
    limits = true;
  }

  @Override
  public void disableLimit() {
    limits = false;
  }
}
