package frc.robot.util.pi;

import edu.wpi.first.networktables.NetworkTable;

public class OrangePIInputReal implements OrangePIInput {
  public void updateInputs() {
    NetworkTable networkTable = NetworkTable.getTable("OrangePI");
  }
  
}
