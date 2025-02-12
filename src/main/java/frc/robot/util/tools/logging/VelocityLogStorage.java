package frc.robot.util.tools.logging;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class VelocityLogStorage implements Loggable {
  private DoubleSupplier currentVelocity;

  public DoubleSupplier getCurrentVelocity() {
    return currentVelocity;
  }

  private DoubleSupplier setpointVelocity;
  private String name;

  public DoubleSupplier getSetpointVelocity() {
    return setpointVelocity;
  }

  public VelocityLogStorage(
      DoubleSupplier currentVelocity, DoubleSupplier setpointVelocity, String name) {
    this.currentVelocity = currentVelocity;
    this.setpointVelocity = setpointVelocity;
    this.name = name;
  }

  @Override
  public void log() {
    Logger.recordOutput("VelocityLogging/" + name + "/setpoint", setpointVelocity.getAsDouble());
    Logger.recordOutput(
        "VelocityLogging/" + name + "/currentVelocity", currentVelocity.getAsDouble());
    Logger.recordOutput("VelocityLogging/" + name + "/velocityError", Math.abs(setpointVelocity.getAsDouble()) - Math.abs(currentVelocity.getAsDouble()));
  }
}
