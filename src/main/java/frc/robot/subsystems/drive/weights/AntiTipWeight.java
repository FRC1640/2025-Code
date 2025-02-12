package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.util.dashboard.PIDMap.PIDKey;
import org.littletonrobotics.junction.Logger;

public class AntiTipWeight implements DriveWeight {
  private Gyro gyro;

  private PIDController pitchAntiTip =
      RobotPIDConstants.constructPID(RobotPIDConstants.driveAntiTip, PIDKey.ANTITIP);
  private PIDController rollAntiTip =
      RobotPIDConstants.constructPID(RobotPIDConstants.driveAntiTip, PIDKey.ANTITIP);

  public AntiTipWeight(Gyro gyro) {

    this.gyro = gyro;
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    double xSpeed = 0;
    double ySpeed = 0;
    if (Math.abs(gyro.getPitch().getDegrees()) > DriveConstants.minTipDegrees) {
      xSpeed = pitchAntiTip.calculate(gyro.getPitch().getRadians(), 0);
    }
    if (Math.abs(gyro.getRoll().getDegrees()) > DriveConstants.minTipDegrees) {
      ySpeed = -rollAntiTip.calculate(gyro.getRoll().getRadians(), 0);
    }

    xSpeed =
        MathUtil.clamp(
            xSpeed,
            -DriveConstants.maxAntiTipCorrectionSpeed,
            DriveConstants.maxAntiTipCorrectionSpeed);
    ySpeed =
        MathUtil.clamp(
            ySpeed,
            -DriveConstants.maxAntiTipCorrectionSpeed,
            DriveConstants.maxAntiTipCorrectionSpeed);

    Translation2d speedsNotRotated = new Translation2d(xSpeed, ySpeed);
    speedsNotRotated = speedsNotRotated.rotateBy(gyro.getAngleRotation2d());

    ChassisSpeeds speeds = new ChassisSpeeds(speedsNotRotated.getX(), speedsNotRotated.getY(), 0);

    Logger.recordOutput("AntiTipSpeeds", speeds);
    return speeds;
  }
}
