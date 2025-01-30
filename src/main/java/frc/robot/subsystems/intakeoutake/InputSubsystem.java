package frc.robot.subsystems.intakeoutake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class InputSubsystem extends SubsystemBase {

  IntakeOutakeIOInputsAutoLogged inputs = new IntakeOutakeIOInputsAutoLogged();
  IntakeOutakeIO io;

  public InputSubsystem(IntakeOutakeIO io) {
    this.io = io;
  }
}
