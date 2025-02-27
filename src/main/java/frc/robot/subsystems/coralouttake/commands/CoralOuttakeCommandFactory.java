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
import frc.robot.util.tools.AllianceManager;
import frc.robot.util.tools.DistanceManager;
import java.util.function.DoubleSupplier;

public class CoralOuttakeCommandFactory {
  CoralOuttakeSubsystem intakeSubsystem;
  public boolean runningBack = false;

  public CoralOuttakeCommandFactory(CoralOuttakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
  }

  public Command setIntakeVoltage(DoubleSupplier voltage) {
    return new RunCommand(
            () -> intakeSubsystem.setIntakeVoltage(voltage.getAsDouble()), intakeSubsystem)
        .finallyDo(() -> intakeSubsystem.setIntakeVoltage(0));
  }

  public void constructTriggers() {
    new Trigger(() -> !intakeSubsystem.hasCoral())
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
                    && !intakeSubsystem.hasCoral()
                    && Robot.getState() != RobotState.AUTONOMOUS)
        .onTrue(retractCoral());
  }

  public Command retractCoral() {
    return (new InstantCommand(() -> runningBack = true)
            .andThen(
                setIntakeVoltage(() -> -0.75)
                    .repeatedly()
                    .until(() -> intakeSubsystem.isCoralDetected()))
            .andThen(new InstantCommand(() -> runningBack = false)))
        .andThen(new InstantCommand(() -> intakeSubsystem.setHasCoral(true)));
  }
}
