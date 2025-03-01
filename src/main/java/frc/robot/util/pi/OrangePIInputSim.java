package frc.robot.util.pi;

import org.littletonrobotics.junction.AutoLog;

public class OrangePIInputSim implements OrangePIInput {
  @AutoLog
  public class OrangePIInputLogged {
    public double temperature = 0;
    public double cpuUsage = 0;
    public double memoryUsage = 0;
    public double diskUsage = 0;      
  }

  public void updateInputs() {
  }
}