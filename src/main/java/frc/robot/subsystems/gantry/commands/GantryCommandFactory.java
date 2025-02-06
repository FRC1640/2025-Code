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
    return gantryApplyVoltageCommand(() -> 2)
        .repeatedly()
        .until(() -> gantrySubsystem.isLimitSwitchPressed())
        .andThen(
            gantryApplyVoltageCommand(() -> -1)
                .repeatedly()
                .until(() -> !gantrySubsystem.isLimitSwitchPressed()))
        .andThen(
            gantryApplyVoltageCommand(() -> 0.5)
                .repeatedly()
                .until(() -> gantrySubsystem.isLimitSwitchPressed()))
        // .andThen(
        //     gantryApplyVoltageCommand(() -> -GantryConstants.gantryHomeFastVoltage)
        //         .repeatedly()
        //         .until(() -> !gantrySubsystem.isLimitSwitchPressed()))
        .andThen(new InstantCommand(() -> gantrySubsystem.resetEncoder()));
  }
  // I dont think the limit switch as the bound is a good idea because then the gantry will be
  // slamming into the limit switch all the time. -> Bad for the limit switch. So I think we
  // should just use the encoder value as the bound.

  public Command gantrySetVelocityCommand(DoubleSupplier velocity) {
    return new RunCommand(
            () -> gantrySubsystem.setVelocity(velocity.getAsDouble()), gantrySubsystem)
        .finallyDo(() -> gantrySubsystem.setGantryVoltage(0));
  }

  public Command gantryDriftCommand() { // TODO: breaks if doesn't detect
    return gantrySetVelocityCommand(
            () ->
                gantrySubsystem.getCarriagePosition() < GantryConstants.gantryLimits.low / 2
                    ? 0.1
                    : -0.1)
        .until(
            () ->
                Math.abs(
                        gantrySubsystem.getCarriagePosition()
                            - GantryConstants.gantryLimits.low / 2)
                    < 0.05)
        .andThen(
            gantrySetVelocityCommand(
                    () ->
                        gantrySubsystem.getCarriagePosition() < GantryConstants.gantryLimits.low / 2
                            ? -0.1
                            : 0.1)
                .until(
                    () ->
                        Math.abs(gantrySubsystem.getCarriagePosition())
                                < GantryConstants.gantryPadding
                            || Math.abs(
                                    gantrySubsystem.getCarriagePosition()
                                        - GantryConstants.gantryLimits.low)
                                < 0.05))
        .repeatedly()
        .until(() -> reefDetector.isDetecting())
        .andThen(
            gantrySetVelocityCommand(() -> 0)
                .until(() -> Math.abs(gantrySubsystem.getGantryVelocity()) < 0.01));
  }

  public void constructTriggers() {}
}
