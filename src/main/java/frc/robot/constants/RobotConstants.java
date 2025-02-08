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
import frc.robot.sensors.resolvers.ResolverVoltageInfo;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.ModuleInfo;
import frc.robot.util.tools.Limits;
import frc.robot.util.tools.RobotSwitch;
import frc.robot.util.tools.RobotSwitchManager.RobotType;
import org.photonvision.simulation.SimCameraProperties;

public class RobotConstants {
  // READ DOCS FOR HOW THE ROBOT TYPE SWITCHERS WORK

  public class RobotDimensions {
    public static final double robotWidth = Units.inchesToMeters(30);
    public static final double robotLength = Units.inchesToMeters(30);
    public static final Translation2d robotXY = new Translation2d(robotWidth / 2, robotLength / 2);
  }

  public class RobotConfigConstants {
    public static final RobotType robotType = RobotType.Deux25;

    // subsystems
    public static final boolean gantrySubsystemEnabled =
        new RobotSwitch<Boolean>(true).addValue(RobotType.Prime24, false).get();

    public static final boolean liftSubsystemEnabled =
        new RobotSwitch<Boolean>(true)
            .addValue(RobotType.Prime24, false)
            .addValue(RobotType.Deux24, false)
            .get();

    public static final boolean coralOuttakeSubsystemEnabled =
        new RobotSwitch<Boolean>(true).addValue(RobotType.Prime24, false).get();

    public static final boolean climberSubsystemEnabled =
        new RobotSwitch<Boolean>(true).addValue(RobotType.Deux24, false).get();
    // sensors
    public static final boolean reefDetectorEnabled =
        new RobotSwitch<Boolean>(true).addValue(RobotType.Prime24, false).get();

    // odometry
    public static final boolean gyroEnabled = new RobotSwitch<Boolean>(true).get();
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

    public static final double maxAntiTipCorrectionSpeed = 1.5;
    public static final double minTipDegrees = 6;
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
    public static final int liftLeaderMotorID = new RobotSwitch<Integer>(0).get();
    public static final int liftFollowerMotorID = new RobotSwitch<Integer>(1).get();
    public static final double gearRatio = 5;
    public static final Limits liftLimits = new Limits(0.0, 2.0);
    public static final double liftMaxSpeed = 0.4;
    public static final double liftMaxAccel = 10;
    public static final TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(liftMaxSpeed, liftMaxAccel);
    public static final double sprocketRadius = Units.inchesToMeters(1.5 / 2);

    public enum CoralPreset {
      Safe(0.1, false),
      LeftL2(0.5, false), // TODO correct numbers
      RightL2(0.5, true),
      LeftL3(1, false),
      RightL3(1, true),
      LeftL4(1.5, false),
      RightL4(1.5, true);

      public final double lift;
      public final boolean right; // Driver Station side perspective

      private CoralPreset(double lift, boolean right) {
        this.lift = lift;
        this.right = right;
      }

      public double getLift() {
        return lift;
      }

      public double getGantry(boolean dsSide) {
        return right ^ dsSide
            ? GantryConstants.gantryLimits.low + GantryConstants.gantryPadding
            : -GantryConstants.gantryPadding;
      }
    }
  }

  public static class ReefDetectorConstants {
    public static final int channel = new RobotSwitch<Integer>(15).get();
    public static final double detectionThresh = 550;
    public static final int averageLength = 20;
    public static final double averagePercentage = 0.8;
    public static final double waitTimeSeconds = 0.1;
  }

  // TODO replace with actual values
  public static class WarningThresholdConstants {
    public static final double maxVortexMotorCurrent = 90;
    public static final double maxNeoMotorCurrent = 80;
    public static final double maxNeo550MotorCurrent = 70;
    public static final double maxMotorTemp = 60; // in degrees celcius
    public static final double minBatteryVoltage = 10.5;
  }

  // TODO replace with actual values
  public static class ClimberConstants {
    public static final int climberLiftMotorID = 0;
    public static final int climberWinch1MotorID = 1;
    public static final int climberWinch2MotorID = 2;

    public static final Limits liftLimits = new Limits(0.0, 1000.0);
    public static final Limits winchLimits = new Limits(0.0, 1000.0);
    public static final ResolverVoltageInfo winchResolverInfo =
        new ResolverVoltageInfo(6, 0, 5, 0, 100, null);
    public static final ResolverVoltageInfo liftResolverInfo =
        new ResolverVoltageInfo(7, 0, 5, 0, 100, null);

    public static final double gearRatio = 5;

    public static final int solenoidForwardChannel = 0;
    public static final int solenoidReverseChannel = 1;
  }

  public static class GantryConstants {
    public static final int gantrySparkID = new RobotSwitch<Integer>(12).get();
    public static final double gantryGearRatio = 27.4;
    public static final double pulleyRadius = Units.inchesToMeters(0.5);
    // left -> right limit
    public static final Limits gantryLimits = new Limits(-0.31, null);
    public static final double gantryPadding = 0.03;
    public static final int gantryLimitSwitchDIOPort = new RobotSwitch<Integer>(4).get();
    public static final double alignSpeed = 0.1;
  }

  public static class CoralOuttakeConstants {
    public static final double gearRatio = 0;
    public static final int intakeSparkID = new RobotSwitch<Integer>(24).get();
    // if you dont update this i will find you // *gulp* // You understand what happens if you don't
    public static final int coralDetectorChannel =
        new RobotSwitch<Integer>(8).get(); // update this too
    public static final double distanceRequired = 2;
    public static final double passiveSpeed = 1;
  }
}
