package frc.robot.subsystems.coralplacer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class CoralPlacerSubsystem extends SubsystemBase {

  CoralPlacerIOInputsAutoLogged inputs = new CoralPlacerIOInputsAutoLogged();
  CoralPlacerIO io;

  public CoralPlacerSubsystem(CoralPlacerIO io) {
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
