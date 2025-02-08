package frc.robot.util.tools;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;

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

  public static boolean onDsSideReef(Supplier<Pose2d> robotPose) {
    Translation2d reefPos =
        chooseFromAlliance(FieldConstants.reefCenterPosBlue, FieldConstants.reefCenterPosRed);
    Translation2d robotToReef = reefPos.minus(robotPose.get().getTranslation());
    boolean dsSide = chooseFromAlliance(robotToReef.getX() > 0, robotToReef.getX() < 0);
    return dsSide;
  }
}
