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
  private boolean threshSet = false;
  private boolean direction = false;

  public GantryCommandFactory(GantrySubsystem gantrySubsystem, ReefDetector reefDetector) {
    this.gantrySubsystem = gantrySubsystem;
    this.reefDetector = reefDetector;
  }

  public Command gantryPIDCommand(DoubleSupplier pos) {
    return new RunCommand(
            () -> gantrySubsystem.setCarriagePosition(pos.getAsDouble()), gantrySubsystem)
        // .until(() -> Math.abs(gantrySubsystem.getCarriagePosition() - pos.getAsDouble()) < 0.01)
        .finallyDo(() -> gantrySubsystem.setGantryVoltage(0));
  }

  public Command gantryApplyVoltageCommand(DoubleSupplier voltage) {
    return new RunCommand(
            () -> gantrySubsystem.setGantryVoltage(voltage.getAsDouble()), gantrySubsystem)
        .finallyDo(() -> gantrySubsystem.setGantryVoltage(0));
  }

  public Command gantryHomeCommand() {
    return gantryApplyVoltageCommand(() -> -2)
        .repeatedly()
        .until(() -> gantrySubsystem.isLimitSwitchPressed())
        .andThen(
            gantryApplyVoltageCommand(() -> 1)
                .repeatedly()
                .until(() -> !gantrySubsystem.isLimitSwitchPressed()))
        .andThen(
            gantryApplyVoltageCommand(() -> -0.5)
                .repeatedly()
                .until(() -> gantrySubsystem.isLimitSwitchPressed()))
        // .andThen(
        //     gantryApplyVoltageCommand(() -> -GantryConstants.gantryHomeFastVoltage)
        //         .repeatedly()
        //         .until(() -> !gantrySubsystem.isLimitSwitchPressed()))
        .andThen(new InstantCommand(() -> gantrySubsystem.resetEncoder()))
        .finallyDo(() -> gantrySubsystem.setLimitEnabled(true))
        .beforeStarting(() -> gantrySubsystem.setLimitEnabled(false));
  }
  // I dont think the limit switch as the bound is a good idea because then the gantry will be
  // slamming into the limit switch all the time. -> Bad for the limit switch. So I think we
  // should just use the encoder value as the bound.

  public Command gantrySetVelocityCommand(DoubleSupplier velocity) {
    return new RunCommand(
            () -> gantrySubsystem.setVelocity(velocity.getAsDouble()), gantrySubsystem)
        .finallyDo(() -> gantrySubsystem.setGantryVoltage(0));
  }

  public Command gantryDriftCommandMinima() {
    return (gantrySetVelocityCommand(
                () ->
                    gantrySubsystem.getCarriagePosition() < GantryConstants.gantryLimitCenter
                        ? GantryConstants.alignSpeed
                        : -GantryConstants.alignSpeed)
            .alongWith(new RunCommand(() -> reefDetector.reefFind()))
            .until(
                () ->
                    Math.abs(
                            gantrySubsystem.getCarriagePosition()
                                - GantryConstants.gantryLimitCenter)
                        < GantryConstants.gantryPadding)
            .andThen(new InstantCommand(() -> threshSet = true))
            .andThen(
                gantrySetVelocityCommand(
                        () ->
                            gantrySubsystem.getCarriagePosition()
                                    < GantryConstants.gantryLimitCenter
                                ? -GantryConstants.alignSpeed
                                : GantryConstants.alignSpeed)
                    .until(
                        () ->
                            Math.abs(
                                        gantrySubsystem.getCarriagePosition()
                                            - GantryConstants.gantryLimits.high)
                                    < GantryConstants.gantryPadding
                                || Math.abs(
                                        gantrySubsystem.getCarriagePosition()
                                            - GantryConstants.gantryLimits.low)
                                    < GantryConstants.gantryPadding)))
        .andThen(new InstantCommand(() -> reefDetector.reefFindReset()))
        .andThen(new InstantCommand(() -> threshSet = false))
        .repeatedly()
        .until(
            () -> reefDetector.getDistanceToReef() < reefDetector.getFoundThresh() + 6 && threshSet)
        .andThen(
            gantrySetVelocityCommand(() -> 0)
                .until(() -> Math.abs(gantrySubsystem.getGantryVelocity()) < 0.01))
        .finallyDo(
            () -> {
              reefDetector.reefFindReset();
              threshSet = false;
            });
  }

  public Command gantryDriftCommandThresh() {
    return (gantrySetVelocityCommand(
                () ->
                    gantrySubsystem.getCarriagePosition() < GantryConstants.gantryLimitCenter
                        ? GantryConstants.alignSpeed
                        : -GantryConstants.alignSpeed)
            .andThen(
                new InstantCommand(
                    () ->
                        direction =
                            gantrySubsystem.getCarriagePosition()
                                < GantryConstants.gantryLimitCenter))
            .until(
                () ->
                    Math.abs(
                            gantrySubsystem.getCarriagePosition()
                                - GantryConstants.gantryLimitCenter)
                        < GantryConstants.gantryPadding)
            .andThen(
                gantrySetVelocityCommand(
                        () -> direction ? -GantryConstants.alignSpeed : GantryConstants.alignSpeed)
                    .andThen(new InstantCommand(() -> direction = !direction))
                    .until(
                        () ->
                            Math.abs(
                                        gantrySubsystem.getCarriagePosition()
                                            - GantryConstants.gantryLimits.high)
                                    < GantryConstants.gantryPadding
                                || Math.abs(
                                        gantrySubsystem.getCarriagePosition()
                                            - GantryConstants.gantryLimits.low)
                                    < GantryConstants.gantryPadding)))
        .repeatedly()
        .until(() -> reefDetector.getDistanceToReef() < 500)
        .andThen(gantrySetVelocityCommand(() -> direction ? 0.05 : -0.05).withTimeout(0.05))
        .andThen(
            gantrySetVelocityCommand(() -> 0)
                .until(() -> Math.abs(gantrySubsystem.getGantryVelocity()) < 0.01));
  }

  public Command runGantryMotionProfile(DoubleSupplier pos) {
    return new InstantCommand(() -> gantrySubsystem.resetGantryMotionProfile())
        .andThen(
            new RunCommand(
                    () -> {
                      gantrySubsystem.runGantryMotionProfile(pos.getAsDouble());
                    },
                    gantrySubsystem)
                .finallyDo(
                    () -> {
                      gantrySubsystem.setGantryVoltage(0);
                      gantrySubsystem.resetGantryMotionProfile();
                    }));
  }

  public Command runGantryVelocityMotionProfile(DoubleSupplier vel) {
    return new InstantCommand(() -> gantrySubsystem.resetGantryVelocityMotionProfile())
        .andThen(
            new RunCommand(
                    () -> {
                      gantrySubsystem.runGantryVelocityMotionProfile(vel.getAsDouble());
                    },
                    gantrySubsystem)
                .finallyDo(
                    () -> {
                      gantrySubsystem.setGantryVoltage(0);
                      gantrySubsystem.resetGantryVelocityMotionProfile();
                    }));
  }

  public void constructTriggers() {}
}
