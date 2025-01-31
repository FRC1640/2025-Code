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

  public double getWinch1MotorPosition() {
    return inputs.winchMotor1Position;
  }

  public double getWinch2MotorPosition() {
    return inputs.winchMotor2Position;
  }

  public double getLiftMotorVelocity() {
    return inputs.liftMotorVelocity;
  }

  public double getWinch1MotorVelocity() {
    return inputs.winchMotor1Velocity;
  }

  public double getWinch2MotorVelocity() {
    return inputs.winchMotor2Velocity;
  }

  public double getLiftMotorCurrent() {
    return inputs.liftMotorCurrent;
  }

  public double getWinch1MotorCurrent() {
    return inputs.winchMotor1Current;
  }

  public double getWinch2MotorCurrent() {
    return inputs.winchMotor2Current;
  }

  public double getLiftMotorVoltage() {
    return inputs.liftMotorVoltage;
  }

  public double getWinch1MotorVoltage() {
    return inputs.winchMotor1Voltage;
  }

  public double getWinch2MotorVoltage() {
    return inputs.winchMotor2Voltage;
  }

  public double getLiftTemperature() {
    return inputs.liftMotorTemperature;
  }

  public double getWinch1Temperature() {
    return inputs.winchMotor1Temperature;
  }

  public double getWinch2Temperature() {
    return inputs.winchMotor2Temperature;
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
