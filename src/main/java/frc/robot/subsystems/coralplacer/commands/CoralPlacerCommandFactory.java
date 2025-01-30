package frc.robot.subsystems.coralplacer.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.coralplacer.CoralPlacerSubsystem;
import java.util.function.DoubleSupplier;

public class CoralPlacerCommandFactory {
  CoralPlacerSubsystem intakeSubsystem;

  public CoralPlacerCommandFactory(CoralPlacerSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
  }

  public Command setIntakeVoltage(DoubleSupplier voltage) {
    return new RunCommand(
            () -> intakeSubsystem.setIntakeVoltage(voltage.getAsDouble()), intakeSubsystem)
        .finallyDo(() -> intakeSubsystem.setIntakeVoltage(0));
  }
}
