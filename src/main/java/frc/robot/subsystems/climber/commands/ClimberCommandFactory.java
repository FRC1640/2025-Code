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

  public Command setElevatorPosPID(DoubleSupplier pos) {
    return new RunCommand(
            () -> climberSubsystem.setClimberElevatorPosition(pos.getAsDouble()), climberSubsystem)
        .finallyDo(() -> climberSubsystem.setClimberElevatorVoltage(0));
  }

  public Command elevatorApplyVoltageCommand(DoubleSupplier voltage) {
    return new RunCommand(
            () -> climberSubsystem.setClimberElevatorVoltage(voltage.getAsDouble()),
            climberSubsystem)
        .finallyDo(() -> climberSubsystem.setClimberElevatorVoltage(0));
  }

  public Command setWinchAnglePID(DoubleSupplier pos) {
    return new RunCommand(
            () -> winchSubsystem.setClimberWinchPosition(pos.getAsDouble()), winchSubsystem)
        .finallyDo(() -> winchSubsystem.setClimberWinchVoltage(0));
  }

  /*
   * public Command setWinchPosPID(DoubleSupplier pos) {
    return new RunCommand(
            () -> winchSubsystem.setClimberWinchPosition(pos.getAsDouble()), winchSubsystem)
        .finallyDo(() -> winchSubsystem.setClimberWinchVoltage(0));
  }
   */

  public Command winchApplyVoltageCommand(DoubleSupplier voltage) {
    return new RunCommand(
            () -> winchSubsystem.setClimberWinchVoltage(voltage.getAsDouble()), winchSubsystem)
        .finallyDo(() -> winchSubsystem.setClimberWinchVoltage(0));
  }

  public Command setClampState(BooleanSupplier isClamped) {
    return new InstantCommand(
        () -> climberSubsystem.setSolenoidState(isClamped.getAsBoolean()),
        climberSubsystem,
        winchSubsystem);
  }

  public Command liftHomeCommand() {
    return elevatorApplyVoltageCommand(() -> 8)
        .repeatedly()
        .until(() -> climberSubsystem.isLimitSwitchPressed())
        .andThen(
            elevatorApplyVoltageCommand(() -> -2)
                .repeatedly()
                .until(() -> !climberSubsystem.isLimitSwitchPressed()))
        .andThen(
            elevatorApplyVoltageCommand(() -> 0.5)
                .repeatedly()
                .until(() -> climberSubsystem.isLimitSwitchPressed()))
        // .andThen(
        //     liftApplyVoltageCommand(() -> -liftConstants.liftHomeFastVoltage)
        //         .repeatedly()
        //         .until(() -> !liftSubsystem.isLimitSwitchPressed()))
        .andThen(new InstantCommand(() -> climberSubsystem.resetEncoder()))
        .finallyDo(() -> climberSubsystem.setLimitsEnabled(true))
        .beforeStarting(() -> climberSubsystem.setLimitsEnabled(false));
  }
}
