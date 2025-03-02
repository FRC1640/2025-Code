package frc.robot.util.pi;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.littletonrobotics.junction.Logger;

public class OrangePILogger {
  public static void logPI() {
    NetworkTableInstance networkInstance = NetworkTableInstance.getDefault();
    NetworkTable networkTable = networkInstance.getTable("Orange PI Diagnostics");
    Logger.recordOutput(
        "OrangePITemp", networkTable.getDoubleTopic("CPU Temp").getEntry(0).getAsDouble());
    Logger.recordOutput(
        "OrangePICPUUsage", networkTable.getDoubleTopic("CPU Usage").getEntry(0).getAsDouble());
    Logger.recordOutput(
        "OrangePIMemUsage", networkTable.getDoubleTopic("Memory Usage").getEntry(0).getAsDouble());
    Logger.recordOutput(
        "OrangePIDiskUsage", networkTable.getDoubleTopic("Disk Usage").getEntry(0).getAsDouble());
  }
}
