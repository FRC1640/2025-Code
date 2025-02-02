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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.ModuleInfo;
import frc.robot.util.tools.Limit;
import frc.robot.util.tools.RobotSwitch;
import frc.robot.util.tools.RobotSwitchManager.RobotType;
import org.photonvision.simulation.SimCameraProperties;

public class RobotConstants {
  // READ DOCS FOR HOW THE ROBOT TYPE SWITCHERS WORK

  public class RobotDimensions {
    public static final double robotWidth = Units.inchesToMeters(36);
    public static final double robotLength = Units.inchesToMeters(36);
    public static final Translation2d robotXY = new Translation2d(robotWidth / 2, robotLength / 2);
  }

  public class RobotConfigConstants {
    // TODO enable gantry, coral outtake, drive, gyro for Deux24
    public static final RobotType robotType = RobotType.Deux25;

    // subsystems
    public static final boolean gantrySubsystemEnabled =
        new RobotSwitch<Boolean>(true)
            .addValue(RobotType.Deux25, false)
            .addValue(RobotType.Prime25, false)
            .get();

    public static final boolean liftSubsystemEnabled =
        new RobotSwitch<Boolean>(true)
            .addValue(RobotType.Deux25, true)
            .addValue(RobotType.Prime25, false)
            .get();

    public static final boolean coralOuttakeSubsystemEnabled =
        new RobotSwitch<Boolean>(true)
            .addValue(RobotType.Deux25, false)
            .addValue(RobotType.Prime25, false)
            .get();
    // sensors
    public static final boolean reefDetectorEnabled =
        new RobotSwitch<Boolean>(true)
            .addValue(RobotType.Deux25, true)
            .addValue(RobotType.Prime25, false)
            .get();
    ;

    // odometry
    public static final boolean gyroEnabled =
        new RobotSwitch<Boolean>(true)
            .addValue(RobotType.Deux25, false)
            .addValue(RobotType.Prime25, false)
            .get();
    ;
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
    public static final double gearRatio = 5;
    public static final Limit liftLimits = new Limit(0, 2);
    public static final double liftMaxSpeed = 0.4;
    public static final double liftMaxAccel = 10;
    public static final TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(liftMaxSpeed, liftMaxAccel);
    public static final double sprocketRadius = Units.inchesToMeters(1.5 / 2);
  }

  public static class ReefDetectorConstants {
    public static final int channel = 15;
    public static final double detectionThresh = 325;
  }

  // TODO replace with actual values
  public static class WarningThresholdConstants {
    // current thresholds are in amps and are currently set at the stall current. Consult with team
    // for actual values later.
    public static final double maxVortexMotorCurrent = 90;
    public static final double maxNeoMotorCurrent = 80;
    public static final double maxNeo550MotorCurrent = 70;
    public static final double maxMotorTemp = 60; // in degrees celcius
    public static final double minBatteryVoltage = 10.5;
  }

  public static class GantryConstants {
    public static final int gantrySparkID = 12;
    public static final double gantryGearRatio = 27; // prototype values
    public static final double pulleyRadius =
        Units.inchesToMeters(0.5); // inches for now / placeholder
    // left -> right limit
    public static final Limit gantryLimits = new Limit(-10000, 10000);
    public static final int gantryLimitSwitchDIOPort = 4;

    public static final double gantryHomeFastVoltage = 6;
    public static final double gantryHomeSlowVoltage = 3;
  }

  public static class CoralOuttakeConstants {
    public static final double gearRatio = 0;
    public static final int intakeSparkID = 24; // if you dont update this i will find you // *gulp*
    public static final int coralDetectorChannel = 25; // update this too
    public static final double distanceRequired = 2;
    public static final double passiveSpeed = 1;
  }
}
