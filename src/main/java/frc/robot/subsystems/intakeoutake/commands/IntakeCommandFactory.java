package frc.robot.subsystems.intakeoutake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.intakeoutake.IntakeSubsystem;
import java.util.function.DoubleSupplier;

public class IntakeCommandFactory {
  IntakeSubsystem intakeSubsystem;

  public IntakeCommandFactory(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
  }

  public Command testSetIntakeVoltage(DoubleSupplier voltage, DoubleSupplier voltage2) {
    return new RunCommand(
            () -> intakeSubsystem.setIntakeVoltage(voltage.getAsDouble() - voltage2.getAsDouble()),
            intakeSubsystem)
        .finallyDo(() -> intakeSubsystem.setIntakeVoltage(0));
  }
}
