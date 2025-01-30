package frc.robot.subsystems.intakeoutake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.intakeoutake.IntakeSubsystem;
import java.util.function.DoubleSupplier;

public class IntakeCommandFactory {
  IntakeSubsystem intakeSubsystem;

  public Command setIntakeVoltage(DoubleSupplier voltage) {
    return new RunCommand(
            () -> intakeSubsystem.setIntakeVoltage(voltage.getAsDouble()), intakeSubsystem)
        .finallyDo(() -> intakeSubsystem.setIntakeVoltage(0));
  }
}
