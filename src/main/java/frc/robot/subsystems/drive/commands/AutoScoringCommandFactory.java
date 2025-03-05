package frc.robot.subsystems.drive.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.RobotConstants.CoralOuttakeConstants;
import frc.robot.constants.RobotConstants.LiftConstants.CoralPreset;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.algae.commands.AlgaeCommandFactory;
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

  public Command gantryAlignCommand(Supplier<CoralPreset> getPreset, BooleanSupplier getDsSide) {
    return gantryCommandFactory.gantryPIDCommand(
        () -> getPreset.get().getGantry(getDsSide.getAsBoolean()));
  }

  public Command autoPlace() {
    return (gantryCommandFactory.gantryDriftCommandThresh())
        .andThen(coralOuttakeCommandFactory.outtake().repeatedly().withTimeout(1))
        .finallyDo(() -> coralOuttakeCommandFactory.outtaking = false);
  }

  public Command placeTrough() {
    return coralOuttakeCommandFactory
        .setIntakeVoltage(() -> 12)
        .repeatedly()
        .until(() -> !coralOuttakeSubsystem.hasCoral())
        .andThen(
            new WaitCommand(0.1)
                .deadlineFor(coralOuttakeCommandFactory.setIntakeVoltage(() -> 12).repeatedly()));
  }

  public Command algaeAutoPickup() {
    return algaeCommandFactory
        .setSolenoidState(() -> true)
        .andThen(algaeCommandFactory.setMotorVoltages(() -> 4, () -> 4))
        .repeatedly()
        .until(() -> algaeSubsystem.hasAlgae());
  }

  public Command outtakeCoralCommand() {
    return coralOuttakeCommandFactory.setIntakeVoltage(
        () -> CoralOuttakeConstants.passiveSpeed * 12);
  }
}
