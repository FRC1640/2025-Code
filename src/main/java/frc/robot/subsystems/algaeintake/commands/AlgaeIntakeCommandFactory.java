package frc.robot.subsystems.algaeintake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.algaeintake.AlgaeIntakeSubsystem;
import java.util.function.DoubleSupplier;

public class AlgaeIntakeCommandFactory {
  AlgaeIntakeSubsystem algaeIntakeSubsystem;

  public AlgaeIntakeCommandFactory(AlgaeIntakeSubsystem algaeIntakeSubsystem) {
    this.algaeIntakeSubsystem = algaeIntakeSubsystem;
  }

  public Command algaeIntakeApplyVoltageCommand(DoubleSupplier voltage) {
    return new RunCommand(
            () -> algaeIntakeSubsystem.setIntakeVoltage(voltage.getAsDouble()),
            algaeIntakeSubsystem)
        .finallyDo(() -> algaeIntakeSubsystem.setIntakeVoltage(0));
  }

  public Command algaeIntakeIntakeAlgaeCommand() {
    return new RunCommand(() -> algaeIntakeSubsystem.setIntakeVoltage(9), algaeIntakeSubsystem)
        .andThen(() -> algaeIntakeSubsystem.setIntakeSolenoidState(true), algaeIntakeSubsystem)
        .andThen(() -> algaeIntakeSubsystem.setIntakeVoltage(0), algaeIntakeSubsystem);
  }
}
