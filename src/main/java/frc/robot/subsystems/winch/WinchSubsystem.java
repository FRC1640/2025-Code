package frc.robot.subsystems.winch;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class WinchSubsystem extends SubsystemBase {
  WinchIO winchIO;
  WinchIOInputsAutoLogged inputs = new WinchIOInputsAutoLogged();

  public WinchSubsystem(WinchIO liftIO) {
    this.winchIO = liftIO;
  }

  @Override
  public void periodic() {

    winchIO.updateInputs(inputs);
    Logger.processInputs("Winch/", inputs);
  }

  public double getWinch1MotorPosition() {
    return inputs.winch1MotorPosition;
  }

  public double getWinch2MotorPosition() {
    return inputs.winch2MotorPosition;
  }

  public double getWinchMotorPosition() {
    return ((inputs.winch1MotorPosition + inputs.winch2MotorPosition) / 2.0);
  }

  public double getWinch1MotorVelocity() {
    return inputs.winch1MotorVelocity;
  }

  public double getWinch2MotorVelocity() {
    return inputs.winch2MotorVelocity;
  }

  public double getWinchMotorVelocity() {
    return ((inputs.winch1MotorVelocity + inputs.winch2MotorVelocity) / 2.0);
  }

  public double getWinch1MotorCurrent() {
    return inputs.winch1MotorCurrent;
  }

  public double getWinch2MotorCurrent() {
    return inputs.winch2MotorCurrent;
  }

  public double getWinchMotorCurrent() {
    return ((inputs.winch1MotorCurrent + inputs.winch2MotorCurrent) / 2.0);
  }

  public double getWinch1MotorVoltage() {
    return inputs.winch1MotorVoltage;
  }

  public double getWinch2MotorVoltage() {
    return inputs.winch2MotorVoltage;
  }

  public double getWinchMotorVoltage() {
    return ((inputs.winch1MotorVoltage + inputs.winch2MotorVoltage) / 2.0);
  }

  public double getWinch1MotorTemperature() {
    return inputs.winch1MotorTemperature;
  }

  public double getWinch2MotorTemperature() {
    return inputs.winch2MotorTemperature;
  }

  public double getWinchMotorTemperature() {
    return ((inputs.winch1MotorTemperature + inputs.winch2MotorTemperature) / 2.0);
  }

  public double getAbsoluteEncoderValue() {
    return inputs.winchAngle;
  }

  public void setClimberWinchPosition(double pos) {
    winchIO.setClimberWinchPosition(pos, inputs);
  }

  public void setClimberWinchAngle(double angle) {
    winchIO.setClimberWinchAngle(angle, inputs);
  }

  public void setClimberWinchVoltage(double voltage) {
    winchIO.setClimberWinchVoltage(voltage, inputs);
  }

  public void setClimberWinch1Voltage(double voltage) {
    winchIO.setClimberWinch1Voltage(voltage, inputs);
  }

  public void setClimberWinch2Voltage(double voltage) {
    winchIO.setClimberWinch2Voltage(voltage, inputs);
  }
}
