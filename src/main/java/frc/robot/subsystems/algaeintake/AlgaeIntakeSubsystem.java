package frc.robot.subsystems.algaeintake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  AlgaeIntakeIO AlgaeIntakeIO;
  AlgaeIntakeIOInputsAutoLogged inputs = new AlgaeIntakeIOInputsAutoLogged();

  public AlgaeIntakeSubsystem(AlgaeIntakeIO AlgaeIntakeIO) {
    this.AlgaeIntakeIO = AlgaeIntakeIO;
  }

  @Override
  public void periodic() {

    AlgaeIntakeIO.updateInputs(inputs);
    Logger.processInputs("Intake/", inputs);
  }

  public void setIntakeVoltage(double voltage) {
    AlgaeIntakeIO.setIntakeMotorVoltage(voltage, inputs);
  }

  public double getIntakeMotorVoltage() {
    return inputs.intakeMotorVoltage;
  }

  public double returnTemp() {
    return inputs.intakeMotorTemperature;
  }

  public boolean getIntakeSolenoidState() {
    return inputs.solenoidForward;
  }

  public void setIntakeSolenoidState(boolean forward) {
    AlgaeIntakeIO.setIntakeSolenoidState(forward, inputs);
  }
}
