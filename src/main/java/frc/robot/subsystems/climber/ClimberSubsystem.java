package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.tools.logging.LogRunner;
import frc.robot.util.tools.logging.VelocityLogStorage;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  ClimberIO climberIO;
  ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public ClimberSubsystem(ClimberIO liftIO) {
    this.climberIO = liftIO;
    LogRunner.addLog(
        new VelocityLogStorage(
            () -> getLiftMotorVelocity(), () -> liftIO.velocitySetpoint(), getName()));
  }

  @Override
  public void periodic() {

    climberIO.updateInputs(inputs);
    Logger.processInputs("Climber/", inputs);
  }

  public double getLiftMotorPosition() {
    return inputs.liftMotorPosition;
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

  public double getLiftMotorVelocity() {
    return inputs.liftMotorVelocity;
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

  public double getLiftMotorCurrent() {
    return inputs.liftMotorCurrent;
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

  public double getLiftMotorVoltage() {
    return inputs.liftMotorVoltage;
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

  public double getLiftTemperature() {
    return inputs.liftMotorTemperature;
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
