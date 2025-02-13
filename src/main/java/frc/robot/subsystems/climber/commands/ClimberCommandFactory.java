package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.winch.WinchSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ClimberCommandFactory {
  ClimberSubsystem climberSubsystem;
  WinchSubsystem winchSubsystem;

  public ClimberCommandFactory(ClimberSubsystem climberSubsystem, WinchSubsystem winchSubsystem) {
    this.climberSubsystem = climberSubsystem;
    this.winchSubsystem = winchSubsystem;
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
            () -> winchSubsystem.setClimberWinchPosition(pos.getAsDouble()), winchSubsystem)
        .finallyDo(() -> winchSubsystem.setClimberWinchVoltage(0));
  }

  public Command climberWinchApplyVoltageCommand(DoubleSupplier voltage) {
    return new RunCommand(
            () -> winchSubsystem.setClimberWinchVoltage(voltage.getAsDouble()), winchSubsystem)
        .finallyDo(() -> winchSubsystem.setClimberWinchVoltage(0));
  }

  public Command climberSetClampState(BooleanSupplier isClamped) {
    return new InstantCommand(
        () -> climberSubsystem.setSolenoidState(isClamped.getAsBoolean()), climberSubsystem);
  }
}
