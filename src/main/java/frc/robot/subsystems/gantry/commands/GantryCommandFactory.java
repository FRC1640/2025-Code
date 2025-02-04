package frc.robot.subsystems.gantry.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
    return new RunCommand(
            () -> gantrySubsystem.setGantryVoltage(GantryConstants.gantryHomeFastVoltage),
            gantrySubsystem)
        .deadlineFor(new WaitUntilCommand(() -> gantrySubsystem.isLimitSwitchPressed()))
        .andThen(
            () -> gantrySubsystem.setGantryVoltage(-1 * GantryConstants.gantryHomeFastVoltage),
            gantrySubsystem)
        .deadlineFor(new WaitUntilCommand(() -> !gantrySubsystem.isLimitSwitchPressed()))
        .andThen(
            () -> gantrySubsystem.setGantryVoltage(GantryConstants.gantryHomeSlowVoltage),
            gantrySubsystem)
        .deadlineFor(new WaitUntilCommand(() -> gantrySubsystem.isLimitSwitchPressed()))
        .andThen(
            () -> gantrySubsystem.setGantryVoltage(-1 * GantryConstants.gantryHomeSlowVoltage),
            gantrySubsystem)
        .deadlineFor(new WaitUntilCommand(() -> !gantrySubsystem.isLimitSwitchPressed()))
        .andThen(() -> gantrySubsystem.resetEncoder(), gantrySubsystem)
        .finallyDo(() -> gantrySubsystem.setGantryVoltage(0));
  }

  public Command gantrySweep(boolean left) {
    return gantryApplyVoltageCommand(() -> (left ? 2 : -2)).until(() -> reefDetector.isDetecting());
  }
}
