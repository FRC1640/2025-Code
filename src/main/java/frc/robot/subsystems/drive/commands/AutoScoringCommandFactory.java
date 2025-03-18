package frc.robot.subsystems.drive.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.Robot.Mode;
import frc.robot.subsystems.climber.commands.ClimberCommandFactory;
import frc.robot.subsystems.coralouttake.CoralOuttakeSubsystem;
import frc.robot.subsystems.coralouttake.commands.CoralOuttakeCommandFactory;
import frc.robot.subsystems.gantry.commands.GantryCommandFactory;
import frc.robot.subsystems.lift.commands.LiftCommandFactory;

public class AutoScoringCommandFactory {
  private GantryCommandFactory gantryCommandFactory;
  private LiftCommandFactory liftCommandFactory;
  private CoralOuttakeCommandFactory coralOuttakeCommandFactory;
  private CoralOuttakeSubsystem coralOuttakeSubsystem;
  private ClimberCommandFactory climberCommandFactory;

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
  }

  public Command autoPlace() {
    return (gantryCommandFactory.gantryDriftCommandThresh())
        .andThen(
            coralOuttakeCommandFactory
                .outtake()
                .repeatedly()
                .until(() -> !coralOuttakeSubsystem.hasCoral()))
        .andThen(coralOuttakeCommandFactory.outtake().repeatedly().withTimeout(0.5))
        .finallyDo(() -> coralOuttakeCommandFactory.outtaking = false);
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
}
