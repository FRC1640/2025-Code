package frc.robot.subsystems.lift.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.lift.LiftSubsystem;

public class LiftCommandFactory {
  LiftSubsystem liftSubsystem;

  public LiftCommandFactory(LiftSubsystem liftSubsystem) {
    this.liftSubsystem = liftSubsystem;
  }

  public Command gantryPIDCommand(double pos) {
    return new RunCommand(() -> liftSubsystem.setLiftPosition(pos), liftSubsystem)
        .finallyDo(() -> liftSubsystem.setLiftVoltage(0));
  }
}
