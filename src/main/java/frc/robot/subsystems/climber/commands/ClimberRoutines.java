package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.RobotConstants.ClimberConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.winch.WinchSubsystem;
import java.util.function.BooleanSupplier;

public class ClimberRoutines {

  private final ClimberCommandFactory climberCommandFactory;
  private ClimberSubsystem climberSubsystem;
  private WinchSubsystem winchSubsystem;

  // Time constants (in seconds)
  private final double afterClampDelay = 0.3;

  // Tolerance (in meters)
  private final double tolerance = 5;

  // State booleans
  public final BooleanSupplier
      liftIsLow =
          () ->
              withinTolerance(
                  climberSubsystem.getLiftMotorPosition(), ClimberConstants.liftLimits.low),
      winchIsLow =
          () ->
              withinTolerance(
                  winchSubsystem.getWinchLeaderMotorPosition(), ClimberConstants.winchLimits.low),
      liftIsHigh =
          () ->
              withinTolerance(
                  climberSubsystem.getLiftMotorPosition(), ClimberConstants.liftLimits.high),
      winchIsVertical =
          () ->
              withinTolerance(
                  winchSubsystem.getWinchLeaderMotorPosition(),
                  ClimberConstants.winchVerticalPosition),
      winchIsHigh =
          () ->
              withinTolerance(
                  winchSubsystem.getWinchLeaderMotorPosition(), ClimberConstants.winchLimits.high);

  public boolean manualOverride;
  // when manualOverride
  // able to cancel manualOverride by activating an autoRoutine

  public ClimberRoutines(ClimberCommandFactory climberCommandFactory) {
    this.climberCommandFactory = climberCommandFactory;
    climberSubsystem = climberCommandFactory.climberSubsystem;
    winchSubsystem = climberCommandFactory.winchSubsystem;
  }

  /**
   * @param position in meters
   * @param target in meters
   * @return whether position is within tolerance of target
   */
  public boolean withinTolerance(double position, double target) {
    return position > target - tolerance && position < target + tolerance;
  }

  /**
   * Ensures lift is down, no coral is in outtake, shuts anti-tip off, within last 15 seconds.
   * Should not take any time to execute under normal conditions
   *
   * @return
   */
  public Command initiatePart0() {
    manualOverride = false;
    // TODO implement
    return new InstantCommand();
  }

  /**
   * Lowers lift and sets arm to vertical position, then umclamps
   *
   * @return
   */
  public Command initiatePart1() {
    manualOverride = false;
    return initiatePart0()
        .andThen(lowerLift().alongWith(unwindArm()))
        .andThen(climberCommandFactory.climberSetClampState(() -> false));
  }

  /**
   * clamps, then lowers arm to min position
   *
   * @return
   */
  public Command initiatePart2() {
    manualOverride = false;
    return Commands.sequence(
            climberCommandFactory.climberSetClampState(() -> true),
            new WaitCommand(afterClampDelay),
            windArm())
        .onlyIf(() -> liftIsLow.getAsBoolean() && winchIsVertical.getAsBoolean());
  }

  /**
   * resets to starting position by first returning to part 1 (safe position), then raising the lift
   * and unwinding the winch
   *
   * @return
   */
  public Command resetClimber() {
    manualOverride = false;
    return initiatePart1().andThen(raiseLift().alongWith(resetArm()));
  }

  /**
   * Stops all current climber auto routines
   *
   * @return
   */
  public Command StopRoutine() {
    return climberCommandFactory
        .climberLiftApplyVoltageCommand(() -> 0)
        .andThen(climberCommandFactory.climberWinchApplyVoltageCommand(() -> 0));
  }

  public Command manualOverride() {
    return StopRoutine().andThen(() -> manualOverride = true);
  }

  /**
   * Lowers lift to lowest position
   *
   * @return
   */
  public Command lowerLift() {
    return climberCommandFactory
        .climberSetLiftPosPID(() -> ClimberConstants.liftLimits.low)
        .repeatedly()
        .until(liftIsLow);
  }

  /**
   * Raises lift to highest position
   *
   * @return
   */
  public Command raiseLift() {
    return climberCommandFactory
        .climberSetLiftPosPID(() -> ClimberConstants.liftLimits.high)
        .repeatedly()
        .until(liftIsHigh);
  }

  /**
   * Unwinds climber arm to vertical position
   *
   * @return
   */
  public Command unwindArm() {
    return climberCommandFactory
        .climberSetWinchPosPID(() -> ClimberConstants.winchVerticalPosition)
        .repeatedly()
        .until(winchIsVertical);
  }

  /**
   * Unwinds climber arm to max position
   *
   * @return
   */
  public Command resetArm() {
    return climberCommandFactory
        .climberSetWinchPosPID(() -> ClimberConstants.winchLimits.high)
        .repeatedly()
        .until(winchIsHigh);
  }

  /**
   * Winds arm to min position
   *
   * @return
   */
  public Command windArm() {
    return climberCommandFactory
        .climberSetWinchPosPID(() -> ClimberConstants.winchLimits.low)
        .repeatedly()
        .until(winchIsLow);
  }

  public boolean isReadyToClamp() {
    return climberSubsystem.getSolenoidState() == false
        && liftIsLow.getAsBoolean()
        && winchIsVertical.getAsBoolean()
        && climberSubsystem.getSensor1()
        && climberSubsystem.getSensor2();
  }
}
