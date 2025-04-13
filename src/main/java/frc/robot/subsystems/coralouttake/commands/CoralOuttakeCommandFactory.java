package frc.robot.subsystems.coralouttake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.Robot.RobotState;
import frc.robot.subsystems.coralouttake.CoralOuttakeSubsystem;
import java.util.function.DoubleSupplier;

public class CoralOuttakeCommandFactory {
  CoralOuttakeSubsystem intakeSubsystem;
  public boolean runningBack = false;
  public boolean outtaking = false;
  boolean setHasCoral = false;
  public boolean ranBack = false;

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
                setIntakeVoltage(() -> 2)
                    .until(
                        () ->
                            !intakeSubsystem.isCoralDetected()
                                && intakeSubsystem.hasCoral()
                                && !intakeSubsystem.guillotineCheck()))
            .andThen(setIntakeVoltage(() -> -1.1).until(() -> intakeSubsystem.isCoralDetected()))
            .andThen(new InstantCommand(() -> runningBack = false)))
        .andThen(new InstantCommand(() -> intakeSubsystem.setHasCoral(true)));
  }

  public void constructTriggers() {
    // new Trigger(
    //         () ->
    //             !intakeSubsystem.isCoralDetected()
    //                 && !ranBack
    //                 && !intakeSubsystem.hasCoral()
    //                 && Robot.getState() != RobotState.AUTONOMOUS)
    //     .debounce(0.01)
    //     .and(
    //         () ->
    //             CoralOuttakeConstants.distanceRequired
    //                 > DistanceManager.getNearestPositionDistance(
    //                     RobotOdometry.instance.getPose("Main"),
    //                     AllianceManager.chooseFromAlliance(
    //                         FieldConstants.coralStationPosBlue,
    // FieldConstants.coralStationPosRed)))
    //     .whileTrue(outtake());

    new Trigger(
            () ->
                intakeSubsystem.isCoralDetected()
                    && !outtaking
                    && !ranBack
                    && Robot.getState() != RobotState.AUTONOMOUS)
        .onTrue(
            runBack()
                .finallyDo(
                    () -> {
                      if (intakeSubsystem.hasCoral() && !intakeSubsystem.guillotineCheck()) {
                        ranBack = true;
                      }
                    }))
        .debounce(0.05);

    new Trigger(() -> !intakeSubsystem.hasCoral())
        .debounce(0.05)
        .onTrue(new InstantCommand(() -> ranBack = false));
  }

  public Command outtake() {
    return setIntakeVoltage(() -> setHasCoral ? 5.5 : 3.5)
        .beforeStarting(
            () -> {
              if (intakeSubsystem.hasCoral()) {
                outtaking = true;
              }
            })
        .beforeStarting(() -> setHasCoral = intakeSubsystem.hasCoral());
  }
}
