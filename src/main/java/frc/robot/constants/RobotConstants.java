package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.ModuleInfo;
import org.photonvision.simulation.SimCameraProperties;

public class RobotConstants {
  public class RobotDimensions {
    public static final double robotWidth = Units.inchesToMeters(36);
    public static final double robotLength = Units.inchesToMeters(36);
    public static final Translation2d robotXY = new Translation2d(robotWidth / 2, robotLength / 2);
  }

  public static Pose2d addRobotDim(Pose2d pose2d) {
    Translation2d translation =
        pose2d
            .getTranslation()
            .minus(
                new Translation2d(RobotDimensions.robotWidth / 2, 0)
                    .rotateBy(pose2d.getRotation()));
    return new Pose2d(translation, pose2d.getRotation());
  }

  public static enum PivotId {
    FL,
    FR,
    BL,
    BR;
  }

  public static class DriveConstants {
    public static final double wheelYPos = Units.inchesToMeters(22.75 / 2);
    public static final double wheelXPos = Units.inchesToMeters(22.75 / 2);
    private static final Translation2d frontLeftLocation = new Translation2d(wheelXPos, wheelYPos);
    private static final Translation2d frontRightLocation =
        new Translation2d(wheelXPos, -wheelYPos);
    public static final Translation2d backLeftLocation = new Translation2d(-wheelXPos, wheelYPos);
    public static final Translation2d backRightLocation = new Translation2d(-wheelXPos, -wheelYPos);
    public static final Translation2d[] positions =
        new Translation2d[] {
          frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
        };
    public static double odometryFrequency = 200.0;
    public static final double driveGearRatio = 116.0 / 15.0;
    public static final double steerGearRatio = ((480.0 / 11.0)) * 1.0166667 * 0.99790377777778;

    public static final double maxSpeed = 4.6;
    public static final double maxNorm =
        DriveSubsystem.computeMaxNorm(DriveConstants.positions, new Translation2d());
    public static final double maxOmega = (maxSpeed / maxNorm);
    public static final double wheelRadius = Units.inchesToMeters(1.8892);

    public static final double accelLimit = 20;
    public static final double deaccelLimit = 11;

    public static final double initalSlope = 3.125;
    public static final double finalSlope = 4.375;

    public static final double maxSteerSpeed = 50; // rad per second

    public static final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
            frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    public static final ModuleInfo FL = new ModuleInfo(PivotId.FL, 3, 2, 0, 45);

    public static final ModuleInfo FR = new ModuleInfo(PivotId.FR, 9, 8, 2, -45);

    public static final ModuleInfo BL = new ModuleInfo(PivotId.BL, 5, 4, 1, 135);

    public static final ModuleInfo BR = new ModuleInfo(PivotId.BR, 7, 6, 3, -135);
  }

  public static class CameraConstants {
    public static final CameraConstant frontCamera =
        new CameraConstant(
            new SimCameraProperties(),
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(29.5 / 2),
                    -Units.inchesToMeters(29.5 / 2 - 8),
                    Units.inchesToMeters(10.5)),
                new Rotation3d()),
            1,
            "Front");

    public static final CameraConstant backCamera =
        new CameraConstant(
            new SimCameraProperties(),
            new Transform3d(
                new Translation3d(0.146, -0.356, 0.406),
                new Rotation3d(0, Math.toRadians(1), Math.PI)),
            1,
            "Back");
    public static final Matrix<N3, N1> defaultDriveStandardDev = VecBuilder.fill(0.1, 0.1, 0.1);
    public static final Matrix<N3, N1> defaultVisionStandardDev = VecBuilder.fill(2, 2, 9999999);
  }

  public static class LiftConstants {
    public static final int liftleaderMotorID = 0;
    public static final int liftfollowerMotorID = 1;
    public static final double gearRatio = 10;

    public static final double liftMax = 10;
    public static final double liftMin = -20;
  }

  public static class CoralDetectorConstants {
    public static final int channel = 5;
  }

  public static class ClimberConstants {
    public static final int climberLiftMotorID = 0;
    public static final int climberWinch1MotorID = 1;
    public static final int climberWinch2MotorID = 2;
    public static final double gearRatio = 5; // figure that out later pls

    public static final double liftMax = 0; // figure out later
    public static final double liftMin = 0; // also figyre out
    public static final double winchMin = 0; // figure smth out
    public static final double minchMax = 0; // you gotta help me here
  }

  public static class GantryConstants {
    public static final int gantrySparkID = 13; // UPDATE
    public static final double gantryGearRatio = 10; // UPDATE
    public static final double pulleyRadiusIn = .5; // inches for now / placeholder
    public static final double leftLimit = 0.0; // change these jawns ( ͡° ͜ʖ ͡°)
    public static final double rightLimit = 12.0; // TODO: Change these jawns
  }
}
