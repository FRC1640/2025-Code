package frc.robot.subsystems.coralouttake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class CoralOuttakeSubsystem extends SubsystemBase {

  CoralOuttakeIOInputsAutoLogged inputs = new CoralOuttakeIOInputsAutoLogged();
  CoralOuttakeIO io;

  public CoralOuttakeSubsystem(CoralOuttakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/", inputs);
  }

  public void stop() {
    io.setIntakeVoltage(0);
  }

  public double returnTemp() {
    return inputs.tempCelcius;
  }

  public double returnAppliedVoltage() {
    return inputs.appliedVoltage;
  }

  public void setIntakeVoltage(double voltage) {
    io.setIntakeVoltage(voltage);
  }
}
