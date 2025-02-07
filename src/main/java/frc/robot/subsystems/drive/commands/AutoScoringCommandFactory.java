package frc.robot.subsystems.drive.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants.LiftConstants.CoralPreset;
import frc.robot.subsystems.gantry.commands.GantryCommandFactory;
import frc.robot.subsystems.lift.commands.LiftCommandFactory;
import java.util.function.Supplier;

public class AutoScoringCommandFactory {
  private GantryCommandFactory gantryCommandFactory;
  private LiftCommandFactory liftCommandFactory;

  public AutoScoringCommandFactory(
      GantryCommandFactory gantryCommandFactory, LiftCommandFactory liftCommandFactory) {
    this.gantryCommandFactory = gantryCommandFactory;
    this.liftCommandFactory = liftCommandFactory;
  }

  public Command gantryAlignCommand(Supplier<CoralPreset> getPreset) {
    return gantryCommandFactory.gantryPIDCommand(() -> getPreset.get().getGantry());
  }
}
