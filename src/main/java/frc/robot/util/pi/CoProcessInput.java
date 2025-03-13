package frc.robot.util.pi;

import frc.robot.util.periodic.PeriodicBase;
import org.littletonrobotics.junction.Logger;

public class CoProcessInput extends PeriodicBase {
  OrangePILogger orangePILogger;
  OrangePIStatAutoLogged orangePIStatAutoLogged = new OrangePIStatAutoLogged();

  public CoProcessInput(OrangePILogger orangePILogger) {
    this.orangePILogger = orangePILogger;
  }

  @Override
  public void periodic() {
    orangePILogger.logPI(orangePIStatAutoLogged);
    Logger.processInputs("OrangePI/", orangePIStatAutoLogged);
  }
}
