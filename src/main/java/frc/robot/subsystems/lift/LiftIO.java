import org.littletonrobotics.junction.AutoLog;

public interface LiftIO extends AutoCloseable {
  @AutoLog
  public static class LiftIOInputs {
    public double liftmotor1Position = 0.0;  
    public double lift = 0.0;   
    public double liftmotor1Velocity = 0.0;   
    public double liftmotor2Velocity = 0.0;   
    public double liftmotor1Current = 0.0;    
    public double liftmotor2Current = 0.0;    
    public double liftmotor1Voltage = 0.0;    
    public double liftmotor2Voltage = 0.0;   
  
  }

  public default void updateInputs(LiftIOInputs inputs) {}

  @Override
  default void close() {}
}
