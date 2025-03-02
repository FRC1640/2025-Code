package frc.robot.util.pi;

import edu.wpi.first.networktables.NetworkTableInstance;

public class OrangePIInputReal implements OrangePIInput {

  @Override
  public void updateInputs(OrangePIInputLogged input) {
    NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
    input.cpuUsage =
        networkTable.getDoubleTopic("Orange PI Diagnostics/CPU Usage").getEntry(0).getAsDouble();
    input.temperature =
        networkTable.getDoubleTopic("Orange PI Diagnostics/CPU Temp").getEntry(0).getAsDouble();
    input.memoryUsage =
        networkTable.getDoubleTopic("Orange PI Diagnostics/Memory Usage").getEntry(0).getAsDouble();
    input.diskUsage =
        networkTable.getDoubleTopic("Orange PI Diagnostics/Disk Usage").getEntry(0).getAsDouble();
  }
}
