package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.RobotConstants.ClimberConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.weights.AntiTipWeight;
import frc.robot.subsystems.winch.WinchSubsystem;
import java.util.function.BooleanSupplier;

public class ClimberRoutines {

  private final ClimberCommandFactory climberCommandFactory;
  private ClimberSubsystem climberSubsystem;
  private WinchSubsystem winchSubsystem;

  // Time constants (in seconds)
  private final double afterClampDelay = 0.3;

  // Tolerance (in meters)
  private final double tolerance = 2;

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
      winchIsClimbed =
          () ->
              withinTolerance(
                  winchSubsystem.getWinchLeaderMotorPosition(),
                  ClimberConstants.winchClimbedPosition),
      winchIsHigh =
          () ->
              withinTolerance(
                  winchSubsystem.getWinchLeaderMotorPosition(), ClimberConstants.winchLimits.high);

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

  public boolean withinTolerance(double position, double target, double tolerance) {
    return position > target - tolerance && position < target + tolerance;
  }

  /**
   * Ensures lift is down, no coral is in outtake, shuts anti-tip off, within last 15 seconds.
   * Should not take any time to execute under normal conditions
   *
   * @return
   */
  public Command initiatePart0() {
    // TODO implement
    return new InstantCommand();
  }

  /**
   * Lowers lift and sets arm to vertical position, then umclamps
   *
   * @return
   */
  public Command setupClimb() {
    return initiatePart0()
        .andThen(lowerLift().alongWith(unwindArm()))
        .andThen(climberCommandFactory.setClampState(() -> false));
  }

  /**
   * clamps, then lowers arm to min position
   *
   * @return
   */
  public Command activateClimb() {
    return Commands.sequence(
            new InstantCommand(() -> AntiTipWeight.setAntiTip(false)),
            climberCommandFactory.setClampState(() -> true),
            new WaitCommand(afterClampDelay),
            windArm())
        .onlyIf(
            () ->
                withinTolerance(
                        winchSubsystem.getWinchLeaderMotorPosition(),
                        ClimberConstants.winchLimits.high,
                        tolerance * 2)
                    && withinTolerance(
                        climberSubsystem.getLiftMotorPosition(),
                        ClimberConstants.liftLimits.low,
                        tolerance * 2))
        .finallyDo(() -> AntiTipWeight.setAntiTip(true));
  }

  /**
   * resets to starting position by first returning to part 1 (safe position), then raising the lift
   * and unwinding the winch
   *
   * @return
   */
  public Command resetClimber() {
    return setupClimb().andThen(raiseLift().alongWith(resetArm()));
  }

  /**
   * Lowers lift to lowest position
   *
   * @return
   */
  public Command lowerLift() {
    return climberCommandFactory
        .setElevatorPosPID(() -> ClimberConstants.liftLimits.low)
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
        .setElevatorPosPID(() -> ClimberConstants.liftLimits.high)
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
        .setWinchPosPID(() -> ClimberConstants.winchLimits.high)
        .repeatedly()
        .until(winchIsHigh);
  }

  /**
   * Unwinds climber arm to max position
   *
   * @return
   */
  public Command resetArm() {
    return climberCommandFactory
        .setWinchPosPID(() -> ClimberConstants.winchLimits.low)
        .repeatedly()
        .until(winchIsLow);
  }

  /**
   * Winds arm to min position
   *
   * @return
   */
  public Command windArm() {
    return climberCommandFactory
        .setWinchPosPID(() -> ClimberConstants.winchClimbedPosition)
        .repeatedly();
    // .until(winchIsClimbed);
  }

  public boolean isReadyToClamp() {
    return climberSubsystem.getSolenoidState() == false
        && withinTolerance(
            winchSubsystem.getWinchLeaderMotorPosition(),
            ClimberConstants.winchLimits.high,
            tolerance * 2)
        && withinTolerance(
            climberSubsystem.getLiftMotorPosition(), ClimberConstants.liftLimits.low, tolerance * 2)
        && climberSubsystem.getSensor1()
        && climberSubsystem.getSensor2();
  }
}
