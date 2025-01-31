package frc.robot.subsystems.coralouttake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.coralouttake.CoralOuttakeSubsystem;
import java.util.function.DoubleSupplier;

public class CoralOuttakeCommandFactory {
  CoralOuttakeSubsystem intakeSubsystem;

  public CoralOuttakeCommandFactory(CoralOuttakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
  }

  public Command setIntakeVoltage(DoubleSupplier voltage) {
    return new RunCommand(
            () -> intakeSubsystem.setIntakeVoltage(voltage.getAsDouble()), intakeSubsystem)
        .finallyDo(() -> intakeSubsystem.setIntakeVoltage(0));
  }
}
