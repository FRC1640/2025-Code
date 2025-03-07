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
  boolean setHasCoral = false;
  boolean ranBack = false;

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
                setIntakeVoltage(() -> -0.75)
                    .repeatedly()
                    .until(() -> intakeSubsystem.isCoralDetected()))
            .andThen(new InstantCommand(() -> runningBack = false)))
        .andThen(new InstantCommand(() -> intakeSubsystem.setHasCoral(true)));
  }

  public void constructTriggers() {
    new Trigger(
            () ->
                !(intakeSubsystem.hasCoral() && !intakeSubsystem.isCoralDetected())
                    && !ranBack
                    && Robot.getState() != RobotState.AUTONOMOUS)
        .debounce(0.01)
        .and(
            () ->
                CoralOuttakeConstants.distanceRequired
                    > DistanceManager.getNearestPositionDistance(
                        RobotOdometry.instance.getPose("Main"),
                        AllianceManager.chooseFromAlliance(
                            FieldConstants.coralStationPosBlue, FieldConstants.coralStationPosRed)))
        .whileTrue(outtake());

    new Trigger(
            () ->
                !intakeSubsystem.isCoralDetected()
                    && intakeSubsystem.hasCoral()
                    && !outtaking
                    && Robot.getState() != RobotState.AUTONOMOUS)
        .onTrue(runBack().finallyDo(() -> ranBack = true));

    new Trigger(() -> !intakeSubsystem.hasCoral())
        .debounce(0.05)
        .onTrue(new InstantCommand(() -> ranBack = false));
  }

  public Command outtake() {
    return setIntakeVoltage(() -> setHasCoral ? 5.5 : 2)
        .beforeStarting(
            () -> {
              if (intakeSubsystem.hasCoral()) {
                outtaking = true;
              }
            })
        .beforeStarting(() -> setHasCoral = intakeSubsystem.hasCoral());
  }
}
