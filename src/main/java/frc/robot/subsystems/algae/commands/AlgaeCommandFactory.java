package frc.robot.subsystems.algae.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.algae.AlgaeSubsystem;

public class AlgaeCommandFactory {
  private final AlgaeSubsystem algaeSubsystem;

  public AlgaeCommandFactory(AlgaeSubsystem algaeSubsystem) {
    this.algaeSubsystem = algaeSubsystem;
  }

  public Command setMotorVoltages(DoubleSupplier leftVoltage, DoubleSupplier rightVoltage) {
    Command c = new RunCommand(
            () -> {
              algaeSubsystem.setVoltage(leftVoltage.getAsDouble(), rightVoltage.getAsDouble());
            }, algaeSubsystem)
        .finallyDo(() -> algaeSubsystem.setVoltage(0, 0));
    c.setName("AlgaeSetMotorVoltages");
    return c;
  }

  public Command setSolenoidState(BooleanSupplier state) {
    Command c = new InstantCommand(
        () -> algaeSubsystem.setSolenoid(state.getAsBoolean()), algaeSubsystem);
    c.setName("SetSolenoidState");
    return c;
  }

  public Command algaePassiveCommand() {
    return setMotorVoltages(this::getPassiveVoltage, this::getPassiveVoltage);
  }

  public double getPassiveVoltage() {
    return algaeSubsystem.hasAlgae() ? 1.2 : 0;
  }

  public Command processCommand() {
    Command c = setMotorVoltages(() -> -5, () -> -5)
        .repeatedly()
        .until(() -> !algaeSubsystem.hasAlgae());
    c.setName("ProcessCommand");
    return c;
  }

  public Command manualIntakeCommand() {
    Command c = setSolenoidState(() -> true)
      .andThen(setMotorVoltages(() -> 4, () -> 4));
    c.setName("ManualAlgaeIntake");
    return c;
  }

  public Command manualOuttakeCommand() {
    Command c = setSolenoidState(() -> true)
      .andThen(setMotorVoltages(() -> -5, () -> -5));
    c.setName("ManualAlgaeOuttake");
    return c;
  }

  public Command manualPassiveCommand() {
    Command c = setMotorVoltages(() -> 1, () -> 1);
    c.setName("ManualAlgaePassive");
    return c;
  }

  public Command manualStowCommand() {
    Command c = setSolenoidState(() -> false);
    c.setName("ManualAlgaeStow");
    return c;
  }
}
