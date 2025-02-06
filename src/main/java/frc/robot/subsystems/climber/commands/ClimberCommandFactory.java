package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.climber.ClimberSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ClimberCommandFactory {
  ClimberSubsystem climberSubsystem;

  public ClimberCommandFactory(ClimberSubsystem climberSubsystem) {
    this.climberSubsystem = climberSubsystem;
  }

  public Command climberSetLiftPosPID(DoubleSupplier pos) {
    return new RunCommand(
            () -> climberSubsystem.setClimberLiftPosition(pos.getAsDouble()), climberSubsystem)
        .finallyDo(() -> climberSubsystem.setClimberLiftVoltage(0));
  }

  public Command climberLiftApplyVoltageCommand(DoubleSupplier voltage) {
    return new RunCommand(
            () -> climberSubsystem.setClimberLiftVoltage(voltage.getAsDouble()), climberSubsystem)
        .finallyDo(() -> climberSubsystem.setClimberLiftVoltage(0));
  }

  public Command climberSetWinchPosPID(DoubleSupplier pos) {
    return new RunCommand(
            () -> climberSubsystem.setClimberWinchPosition(pos.getAsDouble()), climberSubsystem)
        .finallyDo(() -> climberSubsystem.setClimberWinchVoltage(0));
  }

  public Command climberWinchApplyVoltageCommand(DoubleSupplier voltage) {
    return new RunCommand(
            () -> climberSubsystem.setClimberWinchVoltage(voltage.getAsDouble()), climberSubsystem)
        .finallyDo(() -> climberSubsystem.setClimberWinchVoltage(0));
  }

  public Command climberSetClampState(BooleanSupplier isClamped) {
    return new RunCommand(
        () -> climberSubsystem.setSolenoidState(isClamped.getAsBoolean()), climberSubsystem);
  }
}
