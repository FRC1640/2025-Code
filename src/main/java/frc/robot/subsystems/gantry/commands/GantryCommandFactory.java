package frc.robot.subsystems.gantry.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.constants.RobotConstants.GantryConstants;
import frc.robot.sensors.reefdetector.ReefDetector;
import frc.robot.subsystems.gantry.GantrySubsystem;
import java.util.function.DoubleSupplier;

public class GantryCommandFactory {
  GantrySubsystem gantrySubsystem;
  private ReefDetector reefDetector;

  public GantryCommandFactory(GantrySubsystem gantrySubsystem, ReefDetector reefDetector) {
    this.gantrySubsystem = gantrySubsystem;
    this.reefDetector = reefDetector;
  }

  public Command gantryPIDCommand(DoubleSupplier pos) {
    return new RunCommand(
            () -> gantrySubsystem.setCarriagePosition(pos.getAsDouble()), gantrySubsystem)
        .finallyDo(() -> gantrySubsystem.setGantryVoltage(0));
  }

  public Command gantryApplyVoltageCommand(DoubleSupplier voltage) {
    return new RunCommand(
            () -> gantrySubsystem.setGantryVoltage(voltage.getAsDouble()), gantrySubsystem)
        .finallyDo(() -> gantrySubsystem.setGantryVoltage(0));
  }

  public Command gantryHomeCommand() {
    return gantryApplyVoltageCommand(() -> 6)
        .repeatedly()
        .until(() -> gantrySubsystem.isLimitSwitchPressed())
        .andThen(
            gantryApplyVoltageCommand(() -> -0.5)
                .repeatedly()
                .until(() -> !gantrySubsystem.isLimitSwitchPressed()))
        .andThen(
            gantryApplyVoltageCommand(() -> 0.1)
                .repeatedly()
                .until(() -> gantrySubsystem.isLimitSwitchPressed()))
        .andThen(new InstantCommand(() -> gantrySubsystem.resetEncoder()));
  }

  public Command gantryDriftCommand(boolean left) {
    return gantryPIDCommand(() -> (left ? GantryConstants.gantryLimits.low : 0))
        .andThen(
            gantryApplyVoltageCommand(() -> (left ? 2 : -2))
                .until(() -> reefDetector.isDetecting()));
  }

  public void constructTriggers() {}
}
