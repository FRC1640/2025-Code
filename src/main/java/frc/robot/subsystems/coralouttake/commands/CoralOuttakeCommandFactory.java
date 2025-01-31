package frc.robot.subsystems.coralouttake.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants.CoralOuttakeConstants;
import frc.robot.subsystems.coralouttake.CoralOuttakeSubsystem;
import frc.robot.util.tools.AllianceManager;
import frc.robot.util.tools.DistanceManager;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class CoralOuttakeCommandFactory {
  CoralOuttakeSubsystem intakeSubsystem;

  public CoralOuttakeCommandFactory(CoralOuttakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
  }

  public Command setIntakeVoltage(DoubleSupplier voltage) {
    return new RunCommand(
            () -> intakeSubsystem.setIntakeVoltage(voltage.getAsDouble()), intakeSubsystem)
        .finallyDo(() -> intakeSubsystem.setIntakeVoltage(0));
  }

  public Command intakeCoral(double passiveSpeed, Supplier<Pose2d> robotPose) {
    Command command =
        new Command() {
          @Override
          public void execute() {
            if (!intakeSubsystem.isCoralDetected()
                && (CoralOuttakeConstants.distanceRequired
                    > DistanceManager.getNearestPositionDistance(
                        robotPose.get(),
                        AllianceManager.chooseFromAlliance(
                            FieldConstants.coralStationPosBlue,
                            FieldConstants.coralStationPosRed)))) {
              intakeSubsystem.setIntakeVoltage(passiveSpeed);
            } else {
              intakeSubsystem.setIntakeVoltage(0);
            }
          }

          @Override
          public void end(boolean interrupted) {
            intakeSubsystem.setIntakeVoltage(0);
          }

          @Override
          public boolean isFinished() {
            return false;
          }
        };
    command.addRequirements(intakeSubsystem);

    return command;
  }
}
