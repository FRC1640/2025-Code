package frc.robot.util.dashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class Dashboard {

  // private SendableChooser<Command> sysidChooser = new SendableChooser<Command>();
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  public Dashboard() {
    autoInit();
    teleopInit();
  }

  private void autoInit() {
    autoChooser = AutoBuilder.buildAutoChooser();
    ShuffleboardTab autoTab = Shuffleboard.getTab("AUTO");
    autoTab.add(autoChooser).withSize(5, 5).withPosition(1, 1);
  }

  public Command getAutoChooserCommand() {
    return autoChooser.getSelected();
  }

  private void teleopInit() {}

  // private void sysidInit() {
  //   BooleanSupplier startNext = controller.b();
  //   BooleanSupplier cancel = controller.a();
  //   ShuffleboardTab sysidTab = Shuffleboard.getTab("SYSID");
  //   sysidChooser.setDefaultOption("none :/", new WaitCommand(0.1));
  //   sysidChooser.addOption(
  //       "Swerve",
  //       CreateSysidCommand.createCommand(
  //           driveSubsystem::sysIdQuasistatic,
  //           driveSubsystem::sysIdDynamic,
  //           "Swerve",
  //           startNext,
  //           cancel,
  //           () -> driveSubsystem.stop()));
  //   sysidTab.add(sysidChooser).withSize(5, 5).withPosition(1, 1);
  // }
}
