package frc.robot.subsystems.lift.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.constants.RobotConstants.LiftConstants.CoralPreset;
import frc.robot.subsystems.lift.LiftSubsystem;
import java.util.function.DoubleSupplier;

public class LiftCommandFactory {
  LiftSubsystem liftSubsystem;

  public LiftCommandFactory(LiftSubsystem liftSubsystem) {
    this.liftSubsystem = liftSubsystem;
  }

  public Command liftSetPosPID(DoubleSupplier pos) {
    return new RunCommand(() -> liftSubsystem.setLiftPosition(pos.getAsDouble()), liftSubsystem)
        .finallyDo(() -> liftSubsystem.setLiftVoltage(0));
  }

  public Command liftApplyVoltageCommand(DoubleSupplier voltage) {
    return new RunCommand(() -> liftSubsystem.setLiftVoltage(voltage.getAsDouble()), liftSubsystem)
        .finallyDo(() -> liftSubsystem.setLiftVoltage(0));
  }

  public Command liftHomeCommand() {
    return liftApplyVoltageCommand(() -> -2) // correct voltage?
        .repeatedly()
        .until(() -> liftSubsystem.isLimitSwitchPressed())
        .andThen(
            liftApplyVoltageCommand(() -> 1)
                .repeatedly()
                .until(() -> !liftSubsystem.isLimitSwitchPressed()))
        .andThen(
            liftApplyVoltageCommand(() -> -0.5)
                .repeatedly()
                .until(() -> liftSubsystem.isLimitSwitchPressed()))
        .andThen(new InstantCommand(() -> liftSubsystem.resetEncoder()))
        .finallyDo(() -> liftSubsystem.setLimitEnabled(true))
        .beforeStarting(() -> liftSubsystem.setLimitEnabled(false));
  }

  public Command runLiftMotionProfile(DoubleSupplier pos) {
    return new RunCommand(
            () -> {
              liftSubsystem.runLiftMotionProfile(pos.getAsDouble());
            },
            liftSubsystem)
        .finallyDo(
            () -> {
              liftSubsystem.setLiftVoltage(0);
              liftSubsystem.resetLiftMotionProfile();
            });
  }

  public Command runLiftMotionProfile(double pos) {
    return new RunCommand(
            () -> {
              liftSubsystem.runLiftMotionProfile(pos);
            },
            liftSubsystem)
        .finallyDo(
            () -> {
              liftSubsystem.setLiftVoltage(0);
              liftSubsystem.resetLiftMotionProfile();
            });
  }

  public boolean getLiftAtPreset(CoralPreset preset) {
    return liftSubsystem.getMotorPosition() - preset.getLift() < 0.1;
  }
}
