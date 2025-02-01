package frc.robot.util.tools;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceManager {

  /**
   * Returns one of the two parameters based on alliance
   *
   * @param <T> Any type can be passed in
   * @param valueBlue if it is blue alliance, it returns this value
   * @param valueRed if it is red alliance, it returns this value
   * @return the parameter that is according to the team
   */
  public static <T> T chooseFromAlliance(T valueBlue, T valueRed) {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? valueRed : valueBlue;
  }
}
