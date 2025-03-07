package frc.robot.subsystems.drive.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.Robot.Mode;
import frc.robot.constants.RobotConstants.LiftConstants.CoralPreset;
import frc.robot.subsystems.climber.commands.ClimberCommandFactory;
import frc.robot.subsystems.coralouttake.CoralOuttakeSubsystem;
import frc.robot.subsystems.coralouttake.commands.CoralOuttakeCommandFactory;
import frc.robot.subsystems.gantry.commands.GantryCommandFactory;
import frc.robot.subsystems.lift.commands.LiftCommandFactory;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class AutoScoringCommandFactory {
  private GantryCommandFactory gantryCommandFactory;
  private LiftCommandFactory liftCommandFactory;
  private CoralOuttakeCommandFactory coralOuttakeCommandFactory;
  private CoralOuttakeSubsystem coralOuttakeSubsystem;
  private ClimberCommandFactory climberCommandFactory;

  private double presetActive;
  private CoralPreset gantryPresetActive;

  public AutoScoringCommandFactory(
      GantryCommandFactory gantryCommandFactory,
      LiftCommandFactory liftCommandFactory,
      CoralOuttakeCommandFactory coralOuttakeCommandFactory,
      CoralOuttakeSubsystem coralOuttakeSubsystem,
      ClimberCommandFactory climberCommandFactory) {
    this.gantryCommandFactory = gantryCommandFactory;
    this.liftCommandFactory = liftCommandFactory;
    this.coralOuttakeCommandFactory = coralOuttakeCommandFactory;
    this.coralOuttakeSubsystem = coralOuttakeSubsystem;
    this.climberCommandFactory = climberCommandFactory;
  }

  public Command autoPlace() {
    return gantryCommandFactory
        .gantryDriftCommand()
        .andThen(new WaitCommand(0.01))
        .andThen(coralOuttakeCommandFactory.outtake().repeatedly())
        .until(() -> !coralOuttakeSubsystem.hasCoral())
        .andThen(
            new WaitCommand(0.3)
                .deadlineFor(coralOuttakeCommandFactory.setIntakeVoltage(() -> 4).repeatedly()));
  }

  public Command autoPlaceWithOther() {
    return gantryCommandFactory
        .gantryDriftCommand()
        .andThen(new WaitCommand(0.01))
        .andThen(coralOuttakeCommandFactory.outtake().repeatedly())
        .until(() -> !coralOuttakeSubsystem.hasCoral())
        .andThen(
            new WaitCommand(0.1)
                .deadlineFor(coralOuttakeCommandFactory.setIntakeVoltage(() -> 4).repeatedly()));
  }

  public Command homing() {
    return new ConditionalCommand(
        new InstantCommand(),
        gantryCommandFactory
            .gantryHomeCommand()
            .alongWith(liftCommandFactory.liftHomeCommand())
            .alongWith(climberCommandFactory.liftHomeCommand()),
        () -> Robot.getMode() == Mode.SIM);
  }

  public Command runLiftToSafe(boolean algaeMode) {
    return setupAutoPlace(() -> CoralPreset.Safe, algaeMode, () -> false);
  }

  public Command setupAutoPlace(
      Supplier<CoralPreset> coralPreset, boolean algaeMode, BooleanSupplier dsSide) {
    return new InstantCommand(
        () -> {
          (new InstantCommand(
                      () -> {
                        presetActive =
                            algaeMode
                                ? coralPreset.get().getLiftAlgae()
                                : coralPreset.get().getLift();
                        gantryPresetActive = coralPreset.get();
                      })
                  .andThen(liftCommandFactory.runLiftMotionProfile(() -> presetActive).asProxy())
                  .andThen(
                      gantryCommandFactory
                          .gantryAlignCommand(() -> gantryPresetActive, dsSide)
                          .asProxy()))
              .schedule();
        });
  }

  public Command autonAutoPlace(Supplier<CoralPreset> coralPreset, boolean algaeMode) {
    return new InstantCommand(
            () -> {
              presetActive =
                  algaeMode ? coralPreset.get().getLiftAlgae() : coralPreset.get().getLift();
              gantryPresetActive = coralPreset.get();
            })
        .andThen(liftCommandFactory.runLiftMotionProfile(() -> presetActive))
        .andThen(gantryCommandFactory.gantryAlignCommand(() -> gantryPresetActive, () -> true));
  }
}
