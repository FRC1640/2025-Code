package frc.robot.subsystems.algae.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class AlgaeCommandFactory {
  private final AlgaeSubsystem algaeSubsystem;

  public AlgaeCommandFactory(AlgaeSubsystem algaeSubsystem) {
    this.algaeSubsystem = algaeSubsystem;
  }

  public Command setMotorVoltages(DoubleSupplier leftVoltage, DoubleSupplier rightVoltage) {
    return new RunCommand(
            () -> algaeSubsystem.setVoltage(leftVoltage.getAsDouble(), rightVoltage.getAsDouble()),
            algaeSubsystem)
        .finallyDo(() -> algaeSubsystem.setVoltage(0, 0));
  }

  public Command setSolenoidState(BooleanSupplier state) {
    return new InstantCommand(
        () -> algaeSubsystem.setSolenoid(state.getAsBoolean()), algaeSubsystem);
  }

  public Command algaePassiveCommand() {
    return setMotorVoltages(this::getPassiveVoltage, this::getPassiveVoltage);
  }

  public double getPassiveVoltage() {
    return algaeSubsystem.hasAlgae() ? 1.2 : 0;
  }

  public Command processCommand() {
    return setMotorVoltages(() -> -5, () -> -5)
        .repeatedly()
        .until(() -> !algaeSubsystem.hasAlgae());
  }
}
