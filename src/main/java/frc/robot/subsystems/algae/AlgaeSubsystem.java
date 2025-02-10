package frc.robot.subsystems.algae;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class AlgaeSubsystem extends SubsystemBase {
  private AlgaeIO io;
  private AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();

  public AlgaeSubsystem(AlgaeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Algae", inputs);
  }

  public double getIntakeMotorLeftVoltage() {
    return inputs.intakeMotorLeftVoltage;
  }

  public double getIntakeMotorLeftVelocity() {
    return inputs.intakeMotorLeftVelocity;
  }

  public double getIntakeMotorLeftCurrent() {
    return inputs.intakeMotorLeftCurrent;
  }

  public double getIntakeMotorRightVoltage() {
    return inputs.intakeMotorRightVoltage;
  }

  public double getIntakeMotorRightVelocity() {
    return inputs.intakeMotorRightVelocity;
  }

  public double getIntakeMotorRightCurrent() {
    return inputs.intakeMotorRightCurrent;
  }

  public boolean getSolenoid() {
    return inputs.solenoidForward;
  }

  public void setSolenoid(boolean set) {
    io.setSolenoid(set);
  }

  public void setVoltage(double left, double right) {
    io.setVoltage(left, right);
  }
}
