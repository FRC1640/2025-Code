package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.sensors.gyro.Gyro;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class JoystickDriveWeight implements DriveWeight {
  private DoubleSupplier xPercent;
  private DoubleSupplier yPercent;
  private DoubleSupplier omegaPercent;
  private static final double DEADBAND = 0.02;
  private BooleanSupplier slowMode;
  private BooleanSupplier fastMode;
  private boolean enabled = true;
  private BooleanSupplier isFC;
  private Gyro gyro;
  private BooleanSupplier isLimited;

  public JoystickDriveWeight(
      DoubleSupplier xPercent,
      DoubleSupplier yPercent,
      DoubleSupplier omegaPercent,
      BooleanSupplier slowMode,
      BooleanSupplier fastMode,
      BooleanSupplier isFC,
      Gyro gyro,
      BooleanSupplier isLimited) {
    this.xPercent = xPercent;
    this.yPercent = yPercent;
    this.omegaPercent = omegaPercent;
    this.slowMode = slowMode;
    this.fastMode = fastMode;
    this.isFC = isFC;
    this.gyro = gyro;
    this.isLimited = isLimited;
  }

  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    // if (!enabled) {
    //   return new ChassisSpeeds();
    // }
    if (!(RobotState.isTeleop() || RobotState.isTest())) {
      return new ChassisSpeeds();
    }
    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(xPercent.getAsDouble(), yPercent.getAsDouble());
    double omega = MathUtil.applyDeadband(omegaPercent.getAsDouble(), DEADBAND);
    omega = Math.copySign(omega * omega, omega);
    if (linearVelocity.getNorm() != 0 && linearVelocity.getNorm() > 1) {
      linearVelocity = linearVelocity.div(linearVelocity.getNorm());
    }
    omega = MathUtil.clamp(omega, -1, 1);
    double xyMult = 0.65;
    double omegaMult = 0.5;
    if (slowMode.getAsBoolean()) {
      xyMult = 0.3;
      omegaMult = 0.2;
    } else if (fastMode.getAsBoolean()) {
      xyMult = 0.98;
      omegaMult = 0.75;
    }
    double scale = 1;
    ChassisSpeeds speeds =
        new ChassisSpeeds(
                linearVelocity.getX() * DriveConstants.maxSpeed * xyMult,
                linearVelocity.getY() * DriveConstants.maxSpeed * xyMult,
                omega * DriveConstants.maxOmega * omegaMult)
            .times(scale);

    if (!isFC.getAsBoolean()) {
      Translation2d speedsNotRotated =
          new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

      speedsNotRotated =
          speedsNotRotated.rotateBy(
              gyro.getAngleRotation2d().plus(Rotation2d.fromRadians(Math.PI)));

      return new ChassisSpeeds(
          speedsNotRotated.getX(), speedsNotRotated.getY(), speeds.omegaRadiansPerSecond);
    }

    return speeds;
  }

  public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {

    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }
}
