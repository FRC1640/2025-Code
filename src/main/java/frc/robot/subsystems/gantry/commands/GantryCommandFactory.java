package frc.robot.subsystems.gantry.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.gantry.GantrySubsystem;

public class GantryCommandFactory {
  GantrySubsystem gantrySubsystem;

  public GantryCommandFactory(GantrySubsystem gantrySubsystem) {
    this.gantrySubsystem = gantrySubsystem;
  }

  public Command gantryPIDCommand(double pos) {
    return new RunCommand(() -> gantrySubsystem.setCarriagePosition(pos), gantrySubsystem)
        .finallyDo(() -> gantrySubsystem.setGantryVoltage(0));
  }
}
