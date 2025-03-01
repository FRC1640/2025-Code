package frc.robot.subsystems.coralouttake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.Robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants.CoralOuttakeConstants;
import frc.robot.sensors.odometry.RobotOdometry;
import frc.robot.subsystems.coralouttake.CoralOuttakeSubsystem;
import frc.robot.util.misc.AllianceManager;
import frc.robot.util.misc.DistanceManager;
import java.util.function.DoubleSupplier;

public class CoralOuttakeCommandFactory {
  CoralOuttakeSubsystem intakeSubsystem;
  public boolean runningBack = false;
  public boolean outtaking = false;

  public CoralOuttakeCommandFactory(CoralOuttakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
  }

  public Command setIntakeVoltage(DoubleSupplier voltage) {
    return new RunCommand(
            () -> intakeSubsystem.setIntakeVoltage(voltage.getAsDouble()), intakeSubsystem)
        .finallyDo(() -> intakeSubsystem.setIntakeVoltage(0));
  }

  public Command runBack() {
    return (new InstantCommand(() -> runningBack = true)
            .andThen(
                setIntakeVoltage(() -> -1)
                    .repeatedly()
                    .until(() -> intakeSubsystem.isCoralDetected()))
            .andThen(new InstantCommand(() -> runningBack = false)))
        .andThen(new InstantCommand(() -> intakeSubsystem.setHasCoral(true)));
  }

  public void constructTriggers() {
    new Trigger(() -> !intakeSubsystem.hasCoral() && Robot.getState() != RobotState.AUTONOMOUS)
        .and(
            () ->
                CoralOuttakeConstants.distanceRequired
                    > DistanceManager.getNearestPositionDistance(
                        RobotOdometry.instance.getPose("Main"),
                        AllianceManager.chooseFromAlliance(
                            FieldConstants.coralStationPosBlue, FieldConstants.coralStationPosRed)))
        .whileTrue(setIntakeVoltage(() -> CoralOuttakeConstants.passiveSpeed * 12));

    new Trigger(
            () ->
                !intakeSubsystem.isCoralDetected()
                    && intakeSubsystem.hasCoral()
                    && !outtaking
                    && Robot.getState() != RobotState.AUTONOMOUS)
        .onTrue(runBack());
  }

  public Command outtake() {
    return setIntakeVoltage(() -> 3)
        .beforeStarting(
            () -> {
              if (intakeSubsystem.hasCoral()) {
                outtaking = true;
              }
            })
        .finallyDo(() -> outtaking = false);
  }
}
