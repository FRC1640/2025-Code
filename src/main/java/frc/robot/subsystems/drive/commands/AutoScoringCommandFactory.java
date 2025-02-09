package frc.robot.subsystems.drive.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.RobotConstants.GantryConstants;
import frc.robot.constants.RobotConstants.LiftConstants.CoralPreset;
import frc.robot.subsystems.coralouttake.CoralOuttakeSubsystem;
import frc.robot.subsystems.coralouttake.commands.CoralOuttakeCommandFactory;
import frc.robot.subsystems.gantry.commands.GantryCommandFactory;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.subsystems.lift.commands.LiftCommandFactory;
import frc.robot.util.tools.AllianceManager;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class AutoScoringCommandFactory {
  private GantryCommandFactory gantryCommandFactory;
  private LiftCommandFactory liftCommandFactory;
  private CoralOuttakeCommandFactory coralOuttakeCommandFactory;
  private CoralOuttakeSubsystem coralOuttakeSubsystem;
  private LiftSubsystem liftSubsystem;
  private CoralPreset preset;
  private boolean isAcceptingPresetChanges;

  public AutoScoringCommandFactory(
      GantryCommandFactory gantryCommandFactory,
      LiftCommandFactory liftCommandFactory,
      LiftSubsystem liftSubsystem,
      CoralOuttakeCommandFactory coralOuttakeCommandFactory,
      CoralOuttakeSubsystem coralOuttakeSubsystem) {
    this.gantryCommandFactory = gantryCommandFactory;
    this.liftCommandFactory = liftCommandFactory;
    this.liftSubsystem = liftSubsystem;
    this.coralOuttakeCommandFactory = coralOuttakeCommandFactory;
    this.coralOuttakeSubsystem = coralOuttakeSubsystem;
    this.preset = CoralPreset.Safe;
  }

  public Command gantryAlignCommand(Supplier<CoralPreset> getPreset, BooleanSupplier getDsSide) {
    return gantryCommandFactory.gantryPIDCommand(
        () -> getPreset.get().getGantry(getDsSide.getAsBoolean()));
  }

  public Command autoPlace() {
    this.isAcceptingPresetChanges = false;
    return gantryCommandFactory
        .gantryDriftCommand()
        .andThen(new WaitCommand(0.01))
        .andThen(coralOuttakeCommandFactory.setIntakeVoltage(() -> 12).repeatedly())
        .until(() -> !coralOuttakeSubsystem.isCoralDetected())
        .andThen(
            new WaitCommand(0.1)
                .deadlineFor(coralOuttakeCommandFactory.setIntakeVoltage(() -> 12).repeatedly())
                .finallyDo(
                    () ->
                        liftSubsystem.setDefaultCommand(
                            liftCommandFactory.runLiftMotionProfile(
                                () -> CoralPreset.Safe.getLift())))
                .alongWith(
                    gantryCommandFactory.gantryPIDCommand(
                        () -> GantryConstants.gantryLimits.low / 2))
                .alongWith(new InstantCommand(() -> setIsAcceptingPresetChanges(true))));
  }

  public Command setupAutoScore(Supplier<Pose2d> target) {
    return new InstantCommand(
            () ->
                liftSubsystem.setDefaultCommand(
                    liftCommandFactory.runLiftMotionProfile(() -> getPreset().getLift())))
        .alongWith(
            gantryAlignCommand(() -> getPreset(), () -> AllianceManager.onDsSideReef(target)));
  }

  public void setPreset(CoralPreset preset) {
    if (!isAcceptingPresetChanges) {
      return;
    }
    this.preset = preset;
  }

  public CoralPreset getPreset() {
    return preset;
  }

  private void setIsAcceptingPresetChanges(boolean isAcceptingPresetChanges) {
    this.isAcceptingPresetChanges = isAcceptingPresetChanges;
  }
}
