package frc.robot.util.pi;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.littletonrobotics.junction.AutoLog;

public class OrangePILogger {
  @AutoLog
  public static class OrangePIStat {
    public double cpuTemp = 0;
    public double cpuUsage = 0;
    public double memUsage = 0;
    public double diskUsage = 0;
  }

  public OrangePILogger() {}

  public void logPI(OrangePIStat orangePIStat) {
    NetworkTableInstance networkInstance = NetworkTableInstance.getDefault();
    NetworkTable networkTable = networkInstance.getTable("Orange PI Diagnostics");
    orangePIStat.cpuTemp = networkTable.getDoubleTopic("CPU Temp").getEntry(0).getAsDouble();
    orangePIStat.cpuUsage = networkTable.getDoubleTopic("CPU Usage").getEntry(0).getAsDouble();
    orangePIStat.memUsage = networkTable.getDoubleTopic("Memory Usage").getEntry(0).getAsDouble();
    orangePIStat.diskUsage = networkTable.getDoubleTopic("Disk Usage").getEntry(0).getAsDouble();
  }
}
