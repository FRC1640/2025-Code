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
    return inputs.elevatorMotorPosition;
  }

  public double getLiftMotorVelocity() {
    return inputs.elevatorMotorVelocity;
  }

  public double getLiftMotorCurrent() {
    return inputs.elevatorMotorCurrent;
  }

  public double getLiftMotorVoltage() {
    return inputs.elevatorMotorVoltage;
  }

  public double getLiftTemperature() {
    return inputs.elevatorMotorTemperature;
  }

  public boolean getSensor1() {
    return inputs.sensor1;
  }

  public boolean getSensor2() {
    return inputs.sensor2;
  }

  public void setClimberElevatorPosition(double pos) {
    climberIO.setClimberLiftPosition(pos, inputs);
  }

  public void setClimberElevatorVoltage(double voltage) {
    climberIO.setClimberLiftVoltage(voltage, inputs);
  }

  /**
   * @return if the solenoid is in the forward position
   */
  public boolean getSolenoidState() {
    return inputs.solenoidForward;
  }

  public void setSolenoidState(boolean forward) {
    climberIO.setSolenoidState(forward);
  }

  public void resetEncoder() {
    climberIO.resetEncoder();
  }

  public boolean isLimitSwitchPressed() {
    return inputs.isLimitSwitchPressed;
  }

  public void setLimitsEnabled(boolean enable) {
    climberIO.setLimitsEnabled(enable);
  }
}
