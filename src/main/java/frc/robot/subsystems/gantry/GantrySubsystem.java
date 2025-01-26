package frc.robot.subsystems.gantry;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GantrySubsystem extends SubsystemBase {
  GantryIOInputsAutoLogged inputs = new GantryIOInputsAutoLogged();
  GantryIO io;

  public GantrySubsystem(GantryIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public Command gantryPIDCommand(double pos) {
    Command c =
        new Command() {
          @Override
          public void execute() {
            setCarriagePosition(pos);
          }
        };
    return c;
  }

  public Command gantryHomeCommand() {}

  public void stop() {
    io.setGantrySpeedVoltage(0);
  }

  public double getGantryVoltage() {
    return inputs.appliedVoltage;
  }

  public double getCarriagePosition() {
    return inputs.encoderPosition;
  }

  public void setCarriagePosition(double pos) {
    io.setCarriagePosition(pos, inputs);
  }

  public void setGantryVoltage(double voltage) {
    io.setGantrySpeedVoltage(voltage);
  }
}
