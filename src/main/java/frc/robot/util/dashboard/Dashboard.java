package frc.robot.util.dashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.sysid.CreateSysidCommand;
import java.util.function.BooleanSupplier;

public class Dashboard {

  private SendableChooser<Command> sysidChooser = new SendableChooser<Command>();
  private SendableChooser<Integer> testChooser = new SendableChooser<Integer>();
  private static SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private static DriveSubsystem driveSubsystem;
  private static CommandXboxController controller;
  private int testSelected = -1;
  private String meh = "nothing here yet. womp womp";

  // private Dashboard() {}

  public void dashboard(DriveSubsystem driveSubsystem, CommandXboxController controller) {
    Dashboard.driveSubsystem = driveSubsystem;
    Dashboard.controller = controller;
    // testInit();
    autoInit();
    teleopInit();
    sysidInit();
    // pidInit();
  }

  private void autoInit() {
    autoChooser = AutoBuilder.buildAutoChooser();
    ShuffleboardTab autoTab = Shuffleboard.getTab("AUTO");
    autoTab.add(autoChooser).withSize(5, 5).withPosition(1, 1);
  }

  public static Command getAutoChooserCommand() {
    return autoChooser.getSelected();
  }

  private void teleopInit() {
    ShuffleboardTab teleopTab = Shuffleboard.getTab("TELEOP");
    teleopTab.add("eh", meh).withSize(1, 1).withPosition(0, 0);
  }

  private void testInit() {
    ShuffleboardTab testTab = Shuffleboard.getTab("TEST");
    testTab.add(testChooser).withSize(5, 5).withPosition(1, 1);
    // CHOOSER DOES NOT WORK
    testChooser.setDefaultOption("none :/", -1);
    testChooser.addOption("SYSID", 0);
    testChooser.addOption("PID", 1);
    switch (testChooser.getSelected()) {
      case 0:
        if (testSelected != 0) {
          testSelected = 0;
          sysidInit();
        }
        break;

      case 1:
        if (testSelected != 1) {
          testSelected = 1;
          pidInit();
        }
        break;

      case -1:
        if (testSelected != -1) {
          testSelected = -1;
        }
        break;
    }
  }

  private void sysidInit() {
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

  private void pidInit() {
    ShuffleboardTab pidTab = Shuffleboard.getTab("PID");
    pidTab.add("too bad", meh).withSize(1, 1).withPosition(0, 0);
  }
}
