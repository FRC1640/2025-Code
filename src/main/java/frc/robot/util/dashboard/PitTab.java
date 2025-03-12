package frc.robot.util.dashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.coralouttake.CoralOuttakeSubsystem;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class PitTab {
  public static ShuffleboardTab pitTab;
  NetworkTableInstance nt = NetworkTableInstance.getDefault();
  NetworkTable networkTable;
  private ClimberSubsystem climberSubsystem;
  private AlgaeSubsystem algaeSubsystem;
  private CoralOuttakeSubsystem coralSubsystem;
      
        public PitTab(ClimberSubsystem climberSubsystem, CoralOuttakeSubsystem coralSubsystem, AlgaeSubsystem algaeSubsystem) {
          this.climberSubsystem = climberSubsystem;
          this.coralSubsystem = coralSubsystem;
          this.algaeSubsystem = algaeSubsystem;
    
  }

  public void init() {
    pitTab = Shuffleboard.getTab("Pit");
    networkTable = nt.getTable("/Shuffleboard/Pit Testing");
    pitTesterBuild();
  }

  public void pitTesterBuild(){
    pitTab
        .addCamera("BackCamera", "USB Camera 0", "http://10.16.40.52:1186/stream.mjpg")
        .withSize(4, 3)
        .withPosition(5, 1);
    pitTab
        .addBoolean("Left Sensor", () -> climberSubsystem.getSensor1())
        .withSize(2, 1)
        .withPosition(5, 0);
    pitTab
        .addBoolean("Right Sensor", () -> climberSubsystem.getSensor2())
        .withSize(2, 1)
        .withPosition(7, 0);
    pitTab
        .addBoolean("Has Algae?", () -> algaeSubsystem.hasAlgae())
        .withSize(1, 4)
        .withPosition(0, 0);
    pitTab
        .addBoolean("Has Coral?", () -> coralSubsystem.hasCoral())
        .withSize(1, 4)
        .withPosition(1, 0);
  }
}
