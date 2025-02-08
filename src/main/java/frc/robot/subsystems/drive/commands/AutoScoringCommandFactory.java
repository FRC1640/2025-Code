package frc.robot.subsystems.drive.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.RobotConstants.GantryConstants;
import frc.robot.constants.RobotConstants.LiftConstants.CoralPreset;
import frc.robot.subsystems.coralouttake.CoralOuttakeSubsystem;
import frc.robot.subsystems.coralouttake.commands.CoralOuttakeCommandFactory;
import frc.robot.subsystems.gantry.commands.GantryCommandFactory;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.subsystems.lift.commands.LiftCommandFactory;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class AutoScoringCommandFactory {
  private GantryCommandFactory gantryCommandFactory;
  private LiftCommandFactory liftCommandFactory;
  private CoralOuttakeCommandFactory coralOuttakeCommandFactory;
  private CoralOuttakeSubsystem coralOuttakeSubsystem;
  private LiftSubsystem liftSubsystem;

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
  }

  public Command gantryAlignCommand(Supplier<CoralPreset> getPreset, BooleanSupplier getDsSide) {
    return gantryCommandFactory.gantryPIDCommand(
        () -> getPreset.get().getGantry(getDsSide.getAsBoolean()));
  }

  public Command autoPlace() {
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
                        () -> GantryConstants.gantryLimits.low / 2)));
  }
}
