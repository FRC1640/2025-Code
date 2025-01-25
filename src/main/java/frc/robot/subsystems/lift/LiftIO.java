import org.littletonrobotics.junction.AutoLog;

public interface LiftIO extends AutoCloseable {
  @AutoLog
  public static class LiftIOInputs {
    public double motor1Position = 0.0;  
    public double motor2Position = 0.0;   
    public double motor1Velocity = 0.0;   
    public double motor2Velocity = 0.0;   
    public double motor1Current = 0.0;    
    public double motor2Current = 0.0;    
    public double motor1Voltage = 0.0;    
    public double motor2Voltage = 0.0;   
    public boolean topLimitSwitch = false; 
    public boolean bottomLimitSwitch = false; 
  }

  public default void updateInputs(LiftIOInputs inputs) {}

  @Override
  default void close() {}
}
