package frc.robot.subsystems.drive.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.RobotConstants.GantryConstants;
import frc.robot.constants.RobotConstants.LiftConstants.CoralPreset;
import frc.robot.subsystems.gantry.commands.GantryCommandFactory;
import frc.robot.subsystems.lift.commands.LiftCommandFactory;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class AutoScoringCommandFactory {
  private GantryCommandFactory gantryCommandFactory;
  private LiftCommandFactory liftCommandFactory;

  public AutoScoringCommandFactory(
      GantryCommandFactory gantryCommandFactory, LiftCommandFactory liftCommandFactory) {
    this.gantryCommandFactory = gantryCommandFactory;
    this.liftCommandFactory = liftCommandFactory;
  }

  public Command autoalignCoralCommand(
      Supplier<CoralPreset> getPreset, BooleanSupplier getAutoaligned) {
    return liftCommandFactory
        .runLiftMotionProfile(() -> getPreset.get().getLift())
        .alongWith(
            new WaitUntilCommand(
                    () ->
                        getAutoaligned.getAsBoolean()
                            && liftCommandFactory.getLiftAtPreset(getPreset.get()))
                .andThen(
                    gantryCommandFactory.gantryPIDCommand(
                        () ->
                            getPreset.get().getGantryRight()
                                ? GantryConstants.gantryPadding
                                : GantryConstants.gantryLimits.low + GantryConstants.gantryPadding))
                .andThen(gantryCommandFactory.gantryDriftCommand()));
  }
}
