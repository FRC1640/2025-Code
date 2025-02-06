package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.sensors.odometry.RobotOdometry;
import frc.robot.util.tools.AutoAlignHelper;
import org.littletonrobotics.junction.Logger;

public class AntiTipWeight implements DriveWeight {
  private Gyro gyro;

  private PIDController pitchAntiTip =
      RobotPIDConstants.constructPID(RobotPIDConstants.driveAntiTip);
  private PIDController rollAntiTip =
      RobotPIDConstants.constructPID(RobotPIDConstants.driveAntiTip);

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
      ySpeed = rollAntiTip.calculate(gyro.getRoll().getRadians(), 0);
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

    ChassisSpeeds speeds =
        AutoAlignHelper.convertToFieldRelative(
            new ChassisSpeeds(xSpeed, ySpeed, 0), gyro, RobotOdometry.instance.getPose("Main"));

    Logger.recordOutput("AntiTipSpeeds", speeds);
    return new ChassisSpeeds();
  }
}
