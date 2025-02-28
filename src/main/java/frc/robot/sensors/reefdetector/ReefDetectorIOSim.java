package frc.robot.sensors.reefdetector;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class ReefDetectorIOSim implements ReefDetectorIO {

  }

  @Override
  public void updateInputs(ReefDetectorIOInputs inputs) {
    inputs.isConnected = true;

  }
}
