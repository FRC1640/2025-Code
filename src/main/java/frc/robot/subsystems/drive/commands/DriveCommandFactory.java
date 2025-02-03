package frc.robot.subsystems.drive.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.function.Supplier;

public class DriveCommandFactory {
  private DriveSubsystem driveSubsystem;

  public DriveCommandFactory(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
  }

  public Command runVelocityCommand(Supplier<ChassisSpeeds> speeds) {
    return new RunCommand(() -> driveSubsystem.runVelocity(speeds.get(), true, 2.5), driveSubsystem)
        .finallyDo(() -> driveSubsystem.stop());
  }
}
