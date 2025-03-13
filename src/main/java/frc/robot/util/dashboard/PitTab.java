package frc.robot.util.dashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.coralouttake.CoralOuttakeSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.gantry.GantrySubsystem;
import frc.robot.subsystems.lift.LiftSubsystem;
import frc.robot.subsystems.winch.WinchSubsystem;
import frc.robot.util.dashboard.PIDInfo.PIDCommandRegistry;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class PitTab {
  public static ShuffleboardTab pitTab;
  NetworkTableInstance nt = NetworkTableInstance.getDefault();
  NetworkTable networkTable;
  private GantrySubsystem gantrySubsystem;
  private LiftSubsystem liftSubsystem;
  private CoralOuttakeSubsystem coralOuttakeSubsystem;
  private ClimberSubsystem climberSubsystem;
  private AlgaeSubsystem algaeIntakeSubsystem;
  private WinchSubsystem winchSubsystem;

    public PitTab(GantrySubsystem gantrySubsystem, LiftSubsystem liftSubsystem, CoralOuttakeSubsystem coralOuttakeSubsystem, ClimberSubsystem climberSubsystem, AlgaeSubsystem algaeIntakeSubsystem, WinchSubsystem winchSubsystem) {
    this.gantrySubsystem = gantrySubsystem;
    this.liftSubsystem = liftSubsystem;
    this.coralOuttakeSubsystem = coralOuttakeSubsystem;
    this.climberSubsystem = climberSubsystem;
    this.algaeIntakeSubsystem = algaeIntakeSubsystem;
    this.winchSubsystem = winchSubsystem;
    
    }

  public void init() {
    pitTab = Shuffleboard.getTab("Pit");
    networkTable = nt.getTable("/Shuffleboard/Pit Testing");
    pitTesterBuild();
  }

  public void pitTesterBuild(){
    pitTab
        .addCamera("BackCamera", "USB Camera 0", "http://10.16.40.52:1186/stream.mjpg")
        .withSize(2, 2)
        .withPosition(7, 2);
    pitTab
        .addBoolean("Left Sensor", () -> climberSubsystem.getSensor1())
        .withSize(1, 1)
        .withPosition(7, 0);
    pitTab
        .addBoolean("Right Sensor", () -> climberSubsystem.getSensor2())
        .withSize(1, 1)
        .withPosition(8, 0);
    pitTab
        .addBoolean("Has Algae?", () -> algaeIntakeSubsystem.hasAlgae())
        .withSize(1, 1)
        .withPosition(7, 1);
    pitTab
        .addBoolean("Has Coral?", () -> coralOuttakeSubsystem.hasCoral())
        .withSize(1, 1)
        .withPosition(8, 1);
    pitTab
        .add(
            "Cancel All Commands",
            new InstantCommand(
                () -> {}))
        .withPosition(1, 2);
  }
}
