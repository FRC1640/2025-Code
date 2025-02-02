package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
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
  private final ResolverVoltage winchEncoder;
  private final SparkMax liftSpark;
  private final SparkMax winch1Spark;
  private final SparkMax winch2Spark;
  private final PIDController liftPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.climberLiftPID);
  private final PIDController winchPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.climberWinchPID);

  DoubleSolenoid doubleSolenoid;

  public ClimberIOSparkMax() {
    liftSpark =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(ClimberConstants.climberLiftMotorID, false));
    winch1Spark =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(ClimberConstants.climberWinch1MotorID, false));
    winch2Spark =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(ClimberConstants.climberWinch2MotorID, false),
            winch1Spark);
    liftEncoder = new ResolverVoltage(ClimberConstants.winchResolverInfo);
    winchEncoder = new ResolverVoltage(ClimberConstants.winchResolverInfo);
    doubleSolenoid =
        new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            ClimberConstants.solenoidForwardChannel,
            ClimberConstants.solenoidReverseChannel);
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
  /*
   * Set voltage of the winch motors
   */
  @Override
  public void setClimberWinchVoltage(double voltage, ClimberIOInputs inputs) {
    winch1Spark.setVoltage(
        MotorLim.clampVoltage(
            MotorLim.applyLimits(
                inputs.winchMotor1Position, voltage, ClimberConstants.winchLimits)));
  }
  /*
   * Sets the position of the winch motors using a PID
   */
  @Override
  public void setClimberWinchPosition(double position, ClimberIOInputs inputs) {
    setClimberWinchVoltage(
        MotorLim.clampVoltage(winchPID.calculate(inputs.liftMotorPosition, position)), inputs);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.liftMotorPosition = liftEncoder.getDegrees(); // says degrees but really in meters
    inputs.winchMotor1Position = winchEncoder.getDegrees(); // says degrees but really in meters
    inputs.liftMotorCurrent = liftSpark.getOutputCurrent();
    inputs.winchMotor1Current = winch1Spark.getOutputCurrent();
    inputs.winchMotor2Current = winch2Spark.getOutputCurrent();
    inputs.liftMotorVoltage = liftSpark.getAppliedOutput();
    inputs.winchMotor1Voltage = winch1Spark.getAppliedOutput();
    inputs.winchMotor2Voltage = winch2Spark.getAppliedOutput();
    inputs.liftMotorTemperature = liftSpark.getMotorTemperature();
    inputs.winchMotor1Temperature = winch1Spark.getMotorTemperature();
    inputs.winchMotor2Temperature = winch2Spark.getMotorTemperature();
  }

  @Override
  public void setSolenoidState(boolean forward, ClimberIOInputs inputs) {
    if (forward) {
      doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    } else {
      doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }
}
