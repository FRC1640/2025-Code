package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  IntakeIO IntakeIO;
  IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public IntakeSubsystem(IntakeIO IntakeIO) {
    this.IntakeIO = IntakeIO;
  }

  @Override
  public void periodic() {

    IntakeIO.updateInputs(inputs);
    Logger.processInputs("Intake/", inputs);
  }

  public double getLiftMotorPosition() {
    return inputs.intakeMotorPosition;
  }

  public double getintakeMotorVelocity() {
    return inputs.intakeMotorVelocity;
  }

  public double getIntakeMotorCurrent() {
    return inputs.intakeMotorCurrent;
  }

  public double getintakerMotorVoltage() {
    return inputs.intakeMotorVoltage;
  }

  public double getintakeTemperature() {
    return inputs.intakeMotorTemperature;
  }

  public void setIntakePosition(double pos) {
    IntakeIO.setIntakemotor1Position(pos, inputs);
  }

  public boolean getSolenoidState() {
    return inputs.solenoidForward;
  }

  public void setSolenoidState(boolean forward) {
    IntakeIO.setIntakeSolenoidState(forward, inputs);
  }
}
