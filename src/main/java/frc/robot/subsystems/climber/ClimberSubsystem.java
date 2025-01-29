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
    return inputs.winch1MotorPosition;
  }

  public double getWinch2MotorPosition() {
    return inputs.winch2MotorPosition;
  }

  public double getLiftMotorVelocity() {
    return inputs.liftMotorVelocity;
  }

  public double getWinch1MotorVelocity() {
    return inputs.winch1MotorVelocity;
  }

  public double getWinch2MotorVelocity() {
    return inputs.winch2MotorVelocity;
  }

  public double getLiftMotorCurrent() {
    return inputs.liftMotorCurrent;
  }

  public double getWinch1MotorCurrent() {
    return inputs.winch1MotorCurrent;
  }

  public double getWinch2MotorCurrent() {
    return inputs.winch2MotorCurrent;
  }

  public double getLiftMotorVoltage() {
    return inputs.liftMotorVoltage;
  }

  public double getWinch1MotorVoltage() {
    return inputs.winch1MotorVoltage;
  }

  public double getWinch2MotorVoltage() {
    return inputs.winch2MotorVoltage;
  }

  public double getLiftTemperature() {
    return inputs.liftTemperature;
  }

  public double getWinch1Temperature() {
    return inputs.winch1Temperature;
  }

  public double getWinch2Temperature() {
    return inputs.winch2Temperature;
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
}
