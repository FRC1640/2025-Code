package frc.robot.subsystems.drive.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.algae.commands.AlgaeCommandFactory;
import frc.robot.subsystems.coralouttake.CoralOuttakeSubsystem;
import frc.robot.subsystems.coralouttake.commands.CoralOuttakeCommandFactory;
import frc.robot.subsystems.gantry.commands.GantryCommandFactory;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.subsystems.lift.commands.LiftCommandFactory;

public class AutoScoringCommandFactory {
  private GantryCommandFactory gantryCommandFactory;
  private LiftCommandFactory liftCommandFactory;
  private CoralOuttakeCommandFactory coralOuttakeCommandFactory;
  private CoralOuttakeSubsystem coralOuttakeSubsystem;
  private LiftSubsystem liftSubsystem;
  private AlgaeCommandFactory algaeCommandFactory;
  private AlgaeSubsystem algaeSubsystem;

  public AutoScoringCommandFactory(
      GantryCommandFactory gantryCommandFactory,
      LiftCommandFactory liftCommandFactory,
      LiftSubsystem liftSubsystem,
      CoralOuttakeCommandFactory coralOuttakeCommandFactory,
      CoralOuttakeSubsystem coralOuttakeSubsystem,
      AlgaeCommandFactory algaeCommandFactory,
      AlgaeSubsystem algaeSubsystem) {
    this.gantryCommandFactory = gantryCommandFactory;
    this.liftCommandFactory = liftCommandFactory;
    this.liftSubsystem = liftSubsystem;
    this.coralOuttakeCommandFactory = coralOuttakeCommandFactory;
    this.coralOuttakeSubsystem = coralOuttakeSubsystem;
    this.algaeCommandFactory = algaeCommandFactory;
    this.algaeSubsystem = algaeSubsystem;
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
}
