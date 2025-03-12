package frc.robot.util.dashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class PitTab {
  public static ShuffleboardTab pidTab;
  NetworkTableInstance nt = NetworkTableInstance.getDefault();
  NetworkTable networkTable;

  public PitTab() {}

  public void init() {
    pidTab = Shuffleboard.getTab("Pit");
    networkTable = nt.getTable("/Shuffleboard/Pit Testing");
    pitTesterBuild();
  }

  public void pitTesterBuild(){
    
  }
}
