package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.constants.RobotConstants.ClimberConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.constants.SparkConstants;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.tools.MotorLim;

public class ClimberIOSparkMax implements ClimberIO {
  private final RelativeEncoder liftEncoder;
  private final SparkMax liftSpark;
  private final SparkLimitSwitch liftLimitSwitch;
  private final PIDController liftPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.climberLiftPID);

  private final DoubleSolenoid doubleSolenoid;
  // inductance sensors that pull low when metal is detected
  private final DigitalInput sensor1Input, sensor2Input;
  private boolean limits = false;

  public ClimberIOSparkMax() {
    liftSpark =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(ClimberConstants.climberLiftMotorID, false));
    liftEncoder = liftSpark.getEncoder();
    doubleSolenoid =
        new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            ClimberConstants.solenoidForwardChannel,
            ClimberConstants.solenoidReverseChannel);
    sensor1Input = new DigitalInput(ClimberConstants.sensor1Channel);
    sensor2Input = new DigitalInput(ClimberConstants.sensor2Channel);
    liftLimitSwitch = liftSpark.getForwardLimitSwitch();
  }
  /*
   * Set voltage of the lift motor
   */
  @Override
  public void setClimberLiftVoltage(double voltage, ClimberIOInputs inputs) {
    liftSpark.setVoltage(
        MotorLim.clampVoltage(
            MotorLim.applyLimits(
                inputs.elevatorMotorPosition,
                voltage,
                ClimberConstants.liftLimits.low,
                limits ? ClimberConstants.liftLimits.high : null)));
  }
  /*
   * Sets the position of the lift motor using a PID
   */
  @Override
  public void setClimberLiftPosition(double position, ClimberIOInputs inputs) {
    setClimberLiftVoltage(
        MotorLim.clampVoltage(liftPID.calculate(inputs.elevatorMotorPosition, position)), inputs);
  }

  @Override
  public void setSolenoidState(boolean forward) {
    if (forward) {
      doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    } else {
      doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.elevatorMotorPosition =
        liftEncoder.getPosition()
            / ClimberConstants.gearRatio
            * ClimberConstants.pulleyRadius
            * 2
            * Math.PI;
    inputs.elevatorMotorCurrent = liftSpark.getOutputCurrent();
    inputs.elevatorMotorVoltage = liftSpark.getAppliedOutput();
    inputs.elevatorMotorTemperature = liftSpark.getMotorTemperature();

    inputs.solenoidForward = doubleSolenoid.get() == DoubleSolenoid.Value.kForward;
    inputs.sensor1 = !sensor1Input.get();
    inputs.sensor2 = !sensor2Input.get();
    inputs.isLimitSwitchPressed = liftLimitSwitch.isPressed();
  }

  @Override
  public void resetEncoder() {
    liftEncoder.setPosition(0);
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
