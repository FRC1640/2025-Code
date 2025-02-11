package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.constants.RobotConstants.ClimberConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.constants.SparkConstants;
import frc.robot.sensors.resolvers.ResolverVoltage;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.tools.MotorLim;

public class ClimberIOSparkMax implements ClimberIO {
  private final ResolverVoltage liftEncoder;
  private final SparkMax liftSpark;
  private final PIDController liftPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.climberLiftPID);

  private final DoubleSolenoid doubleSolenoid;
  // inductance sensors that pull low when metal is detected
  private final DigitalInput sensor1Input, sensor2Input;

  public ClimberIOSparkMax() {
    liftSpark =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(ClimberConstants.climberLiftMotorID, false));
    liftEncoder = new ResolverVoltage(ClimberConstants.liftResolverInfo);
    doubleSolenoid =
        new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            ClimberConstants.solenoidForwardChannel,
            ClimberConstants.solenoidReverseChannel);
    sensor1Input = new DigitalInput(ClimberConstants.sensor1Channel);
    sensor2Input = new DigitalInput(ClimberConstants.sensor2Channel);
  }
  /*
   * Set voltage of the lift motor
   */
  @Override
  public void setClimberLiftVoltage(double voltage, ClimberIOInputs inputs) {
    liftSpark.setVoltage(
        MotorLim.clampVoltage(
            MotorLim.applyLimits(inputs.liftMotorPosition, voltage, ClimberConstants.liftLimits)));
  }
  /*
   * Sets the position of the lift motor using a PID
   */
  @Override
  public void setClimberLiftPosition(double position, ClimberIOInputs inputs) {
    setClimberLiftVoltage(
        MotorLim.clampVoltage(liftPID.calculate(inputs.liftMotorPosition, position)), inputs);
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
    inputs.liftMotorPosition = liftEncoder.getDegrees(); // says degrees but really in meters
    inputs.liftMotorCurrent = liftSpark.getOutputCurrent();
    inputs.liftMotorVoltage = liftSpark.getAppliedOutput();
    inputs.liftMotorTemperature = liftSpark.getMotorTemperature();

    inputs.solenoidForward = doubleSolenoid.get() == DoubleSolenoid.Value.kForward;
    inputs.sensor1 = !sensor1Input.get();
    inputs.sensor2 = !sensor2Input.get();
  }
}
