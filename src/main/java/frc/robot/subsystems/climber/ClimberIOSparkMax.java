package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
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
  private final SparkMax winchLeaderSpark;
  private final SparkMax winchFollowerSpark;
  private final PIDController liftPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.climberLiftPID);
  private final PIDController winchPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.climberWinchPID);
  private final Servo servo;

  private final DoubleSolenoid doubleSolenoid;

  public ClimberIOSparkMax() {
    liftSpark =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(ClimberConstants.climberLiftMotorID, false));
    winchLeaderSpark =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(ClimberConstants.climberWinch1MotorID, false));
    winchFollowerSpark =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(ClimberConstants.climberWinch2MotorID, false),
            winchLeaderSpark);
    liftEncoder = new ResolverVoltage(ClimberConstants.liftResolverInfo);
    winchEncoder = new ResolverVoltage(ClimberConstants.winchResolverInfo);
    doubleSolenoid =
        new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            ClimberConstants.solenoidForwardChannel,
            ClimberConstants.solenoidReverseChannel);
    servo = new Servo(ClimberConstants.servoChannel);
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
    winchLeaderSpark.setVoltage(
        MotorLim.clampVoltage(
            MotorLim.applyLimits(
                inputs.winchLeaderMotorPosition, voltage, ClimberConstants.winchLimits)));
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
  public void setSolenoidState(boolean forward) {
    if (forward) {
      doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    } else {
      doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }

  @Override
  public void setServoPosition(double position) {
    servo.set(position);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.liftMotorPosition = liftEncoder.getDegrees(); // says degrees but really in meters
    inputs.winchLeaderMotorPosition =
        winchEncoder.getDegrees(); // says degrees but really in meters
    inputs.liftMotorCurrent = liftSpark.getOutputCurrent();
    inputs.winchLeaderMotorCurrent = winchLeaderSpark.getOutputCurrent();
    inputs.winchFollowerMotorCurrent = winchFollowerSpark.getOutputCurrent();
    inputs.liftMotorVoltage = liftSpark.getAppliedOutput();
    inputs.winchLeaderMotorVoltage = winchLeaderSpark.getAppliedOutput();
    inputs.winchFollowerMotorVoltage = winchFollowerSpark.getAppliedOutput();
    inputs.liftMotorTemperature = liftSpark.getMotorTemperature();
    inputs.winchLeaderMotorTemperature = winchLeaderSpark.getMotorTemperature();
    inputs.winchFollowerMotorTemperature = winchFollowerSpark.getMotorTemperature();

    inputs.solenoidForward = doubleSolenoid.get() == DoubleSolenoid.Value.kForward;
    inputs.servoPosition = servo.get();
  }
}
