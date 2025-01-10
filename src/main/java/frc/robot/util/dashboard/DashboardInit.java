package frc.robot.util.dashboard;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.sysid.CreateSysidCommand;
import java.util.function.BooleanSupplier;

public class DashboardInit {
  /*
   * TODO: dashboard with elastic
   *
   * should include:
   * - sysid setup
   * - pid tuner? maybe?
   * - auto setup
   * - in match dashboard
   */

  private static SendableChooser<Command> sysidChooser = new SendableChooser<Command>();
  private static DriveSubsystem driveSubsystem;
  private static CommandXboxController controller;

  private static boolean sysidInit = false;

  public static void dashboard() {
    sysidInit();
  }

  private static void sysidInit() {
    BooleanSupplier startNext = () -> controller.b().getAsBoolean();
    BooleanSupplier cancel = () -> controller.a().getAsBoolean();
    ShuffleboardTab sysidTab = Shuffleboard.getTab("SYSID");
    sysidChooser.setDefaultOption("none :/", new WaitCommand(0.1));
    sysidChooser.addOption(
        "Swerve",
        CreateSysidCommand.createCommand(
            driveSubsystem::sysIdQuasistatic,
            driveSubsystem::sysIdDynamic,
            "Swerve",
            startNext,
            cancel,
            () -> driveSubsystem.stop()));
    sysidTab.add(sysidChooser).withSize(5, 5).withPosition(1, 1);

    sysidInit = true;
  }
}
