package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  ClimberIO climberIO;
  ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public ClimberSubsystem(ClimberIO liftIO) {
    this.climberIO = liftIO;
  }

  @Override
  public void periodic() {

    climberIO.updateInputs(inputs);
    Logger.processInputs("Climber/", inputs);
  }

  public double getLiftMotorPosition() {
    return inputs.liftMotorPosition;
  }

  public double getWinchMotor1Position() {
    return inputs.winchMotor1Position;
  }

  public double getWinchMotor2Position() {
    return inputs.winchMotor2Position;
  }

  public double getWinchMotorPosition() {
    return ((inputs.winchMotor1Position + inputs.winchMotor2Position) / 2.0);
  }

  public double getLiftMotorVelocity() {
    return inputs.liftMotorVelocity;
  }

  public double getWinchMotor1Velocity() {
    return inputs.winchMotor1Velocity;
  }

  public double getWinchMotor2Velocity() {
    return inputs.winchMotor2Velocity;
  }

  public double getWinchMotorVelocity() {
    return ((inputs.winchMotor1Velocity + inputs.winchMotor2Velocity) / 2.0);
  }

  public double getLiftMotorCurrent() {
    return inputs.liftMotorCurrent;
  }

  public double getWinchMotor1Current() {
    return inputs.winchMotor1Current;
  }

  public double getWinchMotor2Current() {
    return inputs.winchMotor2Current;
  }

  public double getWinchMotorCurrent() {
    return ((inputs.winchMotor1Current + inputs.winchMotor2Current) / 2.0);
  }

  public double getLiftMotorVoltage() {
    return inputs.liftMotorVoltage;
  }

  public double getWinchMotor1Voltage() {
    return inputs.winchMotor1Voltage;
  }

  public double getWinchMotor2Voltage() {
    return inputs.winchMotor2Voltage;
  }

  public double getWinchMotorVoltage() {
    return ((inputs.winchMotor1Voltage + inputs.winchMotor2Voltage) / 2.0);
  }

  public double getLiftTemperature() {
    return inputs.liftMotorTemperature;
  }

  public double getWinchMotor1Temperature() {
    return inputs.winchMotor1Temperature;
  }

  public double getWinchMotor2Temperature() {
    return inputs.winchMotor2Temperature;
  }

  public double getWinchMotorTemperature() {
    return ((inputs.winchMotor1Temperature + inputs.winchMotor2Temperature) / 2.0);
  }

  public void setClimberLiftPosition(double pos) {
    climberIO.setClimberLiftPosition(pos, inputs);
  }

  public void setClimberLiftVoltage(double voltage) {
    climberIO.setClimberLiftVoltage(voltage, inputs);
  }

  public void setClimberWinchPosition(double pos) {
    climberIO.setClimberWinchPosition(pos, inputs);
  }

  public void setClimberWinchVoltage(double voltage) {
    climberIO.setClimberWinchVoltage(voltage, inputs);
  }

  public boolean getSolenoidState() {
    return inputs.solenoidForward;
  }

  public void setSolenoidState(boolean forward) {
    climberIO.setSolenoidState(forward, inputs);
  }
}
