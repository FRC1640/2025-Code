package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.RobotConstants.ClimberConstants;

public class ClimberRoutines {

  private final ClimberCommandFactory climberCommandFactory;

  // Time constants
  private final double afterClampDelay = 0.3;

  // State booleans
  public boolean liftIsLow, winchIsLow, liftIsHigh, winchIsHigh; // could be methods
  public boolean ratchetActivated;
  // when ratchetActivated:
  // unable to change clamp state
  // unable to change lift position
  // unable to make winch motors turn in unwind direction
  public boolean manualOverride;
  // when manualOverride
  // able to cancel manualOverride by activating an autoRoutine
  // that's about it

  public ClimberRoutines(ClimberCommandFactory climberCommandFactory) {
    this.climberCommandFactory = climberCommandFactory;
  }

  public Command initiatePart1() {
    // Check for conditions first
    return climberCommandFactory
        .climberSetClampState(() -> false)
        .alongWith(lowerLift(), unwindArm());
    // TODO figure out servo stuff
    // If a certain angle (position) is reached and confirm part 1 done (through manual checking of
    // lift position and clamp down), activate the servo
    // this should also set a boolean that should prevent certain actions etc.
  }

  public Command initiatePart2() {
    // Check for conditions first
    return climberCommandFactory
        .climberSetClampState(() -> true)
        .andThen(new WaitCommand(afterClampDelay))
        .andThen(windArm());
  }

  /**
   * Stops all current climber routines and returns to stable state
   *
   * @return
   */
  public Command CancelRoutine() {
    return StopRoutine();
    // .andThen();
  }

  /**
   * Stops all current climber routines
   *
   * @return
   */
  public Command StopRoutine() {
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
