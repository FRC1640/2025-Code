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

  public double getWinchLeaderMotorPosition() {
    return inputs.winchLeaderMotorPosition;
  }

  public double getWinchFollowerMotorPosition() {
    return inputs.winchFollowerMotorPosition;
  }

  public double getWinchMotorPosition() {
    return ((inputs.winchLeaderMotorPosition + inputs.winchFollowerMotorPosition) / 2.0);
  }

  public double getWinchLeaderMotorVelocity() {
    return inputs.winchLeaderMotorVelocity;
  }

  public double getWinchFollowerMotorVelocity() {
    return inputs.winchFollowerMotorVelocity;
  }

  public double getWinchMotorVelocity() {
    return ((inputs.winchLeaderMotorVelocity + inputs.winchFollowerMotorVelocity) / 2.0);
  }

  public double getWinchLeaderMotorCurrent() {
    return inputs.winchLeaderMotorCurrent;
  }

  public double getWinchFollowerMotorCurrent() {
    return inputs.winchFollowerMotorCurrent;
  }

  public double getWinchMotorCurrent() {
    return ((inputs.winchLeaderMotorCurrent + inputs.winchFollowerMotorCurrent) / 2.0);
  }

  public double getWinchLeaderMotorVoltage() {
    return inputs.winchLeaderMotorVoltage;
  }

  public double getWinchFollowerMotorVoltage() {
    return inputs.winchFollowerMotorVoltage;
  }

  public double getWinchMotorVoltage() {
    return ((inputs.winchLeaderMotorVoltage + inputs.winchFollowerMotorVoltage) / 2.0);
  }

  public double getWinchLeaderMotorTemperature() {
    return inputs.winchLeaderMotorTemperature;
  }

  public double getWinchFollowerMotorTemperature() {
    return inputs.winchFollowerMotorTemperature;
  }

  public double getWinchMotorTemperature() {
    return ((inputs.winchLeaderMotorTemperature + inputs.winchFollowerMotorTemperature) / 2.0);
  }

  public void setClimberWinchPosition(double pos) {
    winchIO.setClimberWinchPosition(pos, inputs);
  }

  public void setClimberWinchVoltage(double voltage) {
    winchIO.setClimberWinchVoltage(voltage, inputs);
  }
}
