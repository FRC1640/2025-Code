package frc.robot.subsystems.gantry;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class GantrySubsystem extends SubsystemBase {
  GantryIOInputsAutoLogged inputs = new GantryIOInputsAutoLogged();
  GantryIO io;

  public GantrySubsystem(GantryIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Gantry/", inputs);
  }

  public void stop() {
    io.setGantryVoltage(0, inputs);
  }

  public double getGantryVoltage() {
    return inputs.appliedVoltage;
  }

  public double getCarriagePosition() {
    return inputs.encoderPosition;
  }

  public void setCarriagePosition(double pos) {
    io.setGantryPosition(pos, inputs);
  }

  public void setGantryVoltage(double voltage) {
    io.setGantryVoltage(voltage, inputs);
  }
}
