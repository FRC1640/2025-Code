package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.RobotConstants.ClimberConstants;

public class ClimberRoutines {

  private final ClimberCommandFactory climberCommandFactory;

  // Time constants
  private final double afterClampDelay = 0.3;

  public ClimberRoutines(ClimberCommandFactory climberCommandFactory) {
    this.climberCommandFactory = climberCommandFactory;
  }

  public Command initiatePart1() {
    // Check for conditions first
    return climberCommandFactory
        .climberSetClampState(() -> false)
        .alongWith(lowerLift(), unwindArm());
  }

  public Command initiatePart2() {
    // Check for conditions first
    return climberCommandFactory
        .climberSetClampState(() -> true)
        .andThen(new WaitCommand(afterClampDelay))
        .andThen(windArm());
  }

  /**
   * Stops all current climber routines
   *
   * @return
   */
  public Command CancelRoutine() {
    return null;
  }

  /**
   * Lowers lift to lowest position
   *
   * @return
   */
  public Command lowerLift() {
    return climberCommandFactory.climberSetLiftPosPID(() -> ClimberConstants.liftLimits.low);
  }

  /**
   * Raises lift to highest position
   *
   * @return
   */
  public Command raiseLift() {
    return climberCommandFactory.climberSetLiftPosPID(() -> ClimberConstants.liftLimits.high);
  }

  /**
   * Unwinds climber arm to max position
   *
   * @return
   */
  public Command unwindArm() {
    return climberCommandFactory.climberSetWinchPosPID(() -> ClimberConstants.winchLimits.high);
  }

  /**
   * Winds arm to min position
   *
   * @return
   */
  public Command windArm() {
    return climberCommandFactory.climberSetWinchPosPID(() -> ClimberConstants.winchLimits.low);
  }
}
