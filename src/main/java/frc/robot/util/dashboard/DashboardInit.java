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
  private static SendableChooser<String> testChooser = new SendableChooser<String>();
  // private static SendableChooser<Command> testChooser = new SendableChooser<Command>();
  private static String testModes = "";
  private static DriveSubsystem driveSubsystem;
  private static CommandXboxController controller;

  public static void dashboard() {
    testInit();
  }

  private static void testInit() {
    ShuffleboardTab testTab = Shuffleboard.getTab("TEST");
    testTab.add(testChooser).withSize(5, 5).withPosition(1, 1);
    testChooser.setDefaultOption(
        "none :/",
        testModes = ""); // does this actually indicate setting testmodes to blank? need to test
    testChooser.addOption("SYSID", testModes = "sys");
    testChooser.addOption("PID", testModes = "pid");
    switch (testModes) {
      case "sys":
        sysidInit();
        testModes = "";
        break;

      case "pid":
        pidInit();
        testModes = "";
        break;
    }
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
  }

  private static void pidInit() {
    System.out.println("nothin here yet. womp womp");
  }
}
