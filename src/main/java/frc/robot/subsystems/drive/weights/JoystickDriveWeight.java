package frc.robot.subsystems.drive.weights;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.subsystems.drive.commands.DriveWeight;
import java.util.function.DoubleSupplier;

public class JoystickDriveWeight implements DriveWeight {
  private DoubleSupplier xPercent;
  private DoubleSupplier yPercent;
  private DoubleSupplier omegaPercent;
  private Gyro gyro;
  private static final double DEADBAND = 0.02;

  public JoystickDriveWeight(
      DoubleSupplier xPercent, DoubleSupplier yPercent, DoubleSupplier omegaPercent, Gyro gyro) {
    this.xPercent = xPercent;
    this.yPercent = yPercent;
    this.omegaPercent = omegaPercent;
    this.gyro = gyro;
  }

  @Override
  public ChassisSpeeds getSpeeds() {
    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(xPercent.getAsDouble(), yPercent.getAsDouble());
    double omega = MathUtil.applyDeadband(omegaPercent.getAsDouble(), DEADBAND);
    omega = Math.copySign(omega * omega, omega);

    // Convert to field relative speeds & send command
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * DriveConstants.maxSpeed,
            linearVelocity.getY() * DriveConstants.maxSpeed,
            omega * DriveConstants.maxOmega);
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
