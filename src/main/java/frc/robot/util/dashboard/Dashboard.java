package frc.robot.util.dashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
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

  private DriveSubsystem driveSubsystem;
  private CommandXboxController controller;

  private static SendableChooser<Command> sysidChooser = new SendableChooser<Command>();
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  boolean test1 = false, test2 = false, test3 = true; // placeholder booleans >:)
  int test5 = 2; // placeholder integers :O
  String test6 = "PHOTON VISION FEED GOES HERE"; // placeholder string >:(

  public Dashboard(DriveSubsystem driveSubsystem, CommandXboxController controller) {
    this.driveSubsystem = driveSubsystem;
    this.controller = controller;
    autoInit();
    teleopInit();
    mainInit();
    sysidInit();
  }

  private void autoInit() {
    autoChooser = AutoBuilder.buildAutoChooser();
    ShuffleboardTab autoTab = Shuffleboard.getTab("AUTO");
    autoTab.add(autoChooser).withSize(5, 5).withPosition(1, 1);
  }

  private void mainInit() {
    ShuffleboardTab mainTab = Shuffleboard.getTab("MAIN");
    mainTab
        .addInteger(
            "Time Left in Match:", () -> Math.round(DriverStation.getMatchTime() * 10000) / 10000)
        .withSize(5, 3)
        .withPosition(0, 0);
    mainTab.addBoolean("Coral in Bay?", () -> test1).withSize(5, 3).withPosition(0, 3);
    mainTab.addBoolean("Algae in Bay?", () -> test2).withSize(5, 3).withPosition(0, 6);
    mainTab.addString("Photon Vision", () -> test6).withSize(9, 9).withPosition(5, 0);
    mainTab.addInteger("Height of Coral Getter", () -> test5).withSize(5, 3).withPosition(14, 0);
    mainTab
        .addBoolean("Robot Getting More Resources?", () -> test3)
        .withSize(5, 3)
        .withPosition(14, 3);
    mainTab
        .addBoolean("This is here to make all this look nice.", () -> true)
        .withSize(5, 3)
        .withPosition(14, 6);
  }

  public Command getAutoChooserCommand() {
    return autoChooser.getSelected();
  }

  private void teleopInit() {}

  private void sysidInit() {
    BooleanSupplier startNext = controller.b();
    BooleanSupplier cancel = controller.a();
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

  public static Command getSysidCommand() {
    return sysidChooser.getSelected();
  }
}
