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
   * Lowers lift and sets arm to vertical position, then umclamps
   *
   * @return
   */
  public Command setupClimb() {
    return climberCommandFactory
        .setClampState(() -> false)
        .andThen(lowerLift().alongWith(new WaitCommand(0.5).andThen(unwindArm())))
        .repeatedly();
  }

  /**
   * clamps, then lowers arm to min position
   *
   * @return
   */
  public Command activateClimb() {
    return Commands.sequence(
            new InstantCommand(() -> AntiTipWeight.setAntiTipEnabled(false)),
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
        .finallyDo(() -> AntiTipWeight.setAntiTipEnabled(true));
  }

  /**
   * Lowers lift to lowest position
   *
   * @return
   */
  public Command lowerLift() {
    return climberCommandFactory.setElevatorPosPID(() -> ClimberConstants.liftLimits.low);
    // .until(liftIsLow)
  }
  /**
   * Unwinds climber arm to vertical position
   *
   * @return
   */
  public Command unwindArm() {
    return climberCommandFactory.setWinchPosPID(() -> 84.3).repeatedly();
    // .until(winchIsHigh);
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
