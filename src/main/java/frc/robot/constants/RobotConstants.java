package frc.robot.constants;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
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
import frc.robot.util.ConfigEnums.TestMode.TestingSetting;
import frc.robot.util.WPICal.AprilTagPositionSwitcher.AprilTagSetting;
import frc.robot.util.logging.MotorLoggingManager;
import frc.robot.util.misc.Limits;
import frc.robot.util.robotswitch.RobotSwitch;
import frc.robot.util.robotswitch.RobotSwitchManager.RobotType;
import org.photonvision.simulation.SimCameraProperties;

public class RobotConstants {
  // READ DOCS FOR HOW THE ROBOT TYPE SWITCHERS WORK

  public class RobotDimensions {
    public static final double robotWidth = 0.7;
    public static final double robotLength = 0.7; // 0.927
    public static final Translation2d robotXY = new Translation2d(robotWidth / 2, robotLength / 2);
  }

  public class TestConfig {
    public static final TestingSetting testingMode = TestingSetting.pit;
  }

  public class MotorInfo {
    public static final MotorLoggingManager motorLoggingManager =
        new MotorLoggingManager()
            .addMotorAlias(GantryConstants.gantrySparkID, "Gantry")
            .addMotorAlias(ClimberConstants.climberLiftMotorID, "Climber Lift")
            .addMotorAlias(ClimberConstants.climberWinch1MotorID, "Climber Winch 1 (Leader)")
            .addMotorAlias(ClimberConstants.climberWinch2MotorID, "Climber Winch 2 (Follower)")
            .addMotorAlias(AlgaeConstants.motorLeftChannel, "Algae Motor Left")
            .addMotorAlias(AlgaeConstants.motorRightChannel, "Algae Motor Right")
            .addMotorAlias(LiftConstants.liftLeaderMotorID, "Lift Leader")
            .addMotorAlias(LiftConstants.liftFollowerMotorID, "Lift Follower")
            .addMotorAlias(CoralOuttakeConstants.intakeSparkID, "Coral Outtake");
  }

  public class PneumaticsConstants {
    public static final int pneumaticsHubID = 22;
  }

  public class AprilTagPositionSettings {
    public static final AprilTagSetting fieldPositionType = AprilTagSetting.WPILibWelded;
    public static final String WPICalOutputJson = "ImportedLayout.json";
  }

  public class RobotConfigConstants {
    public static final RobotType robotType = RobotType.Prime24;
    // subsystems
    public static final boolean gantrySubsystemEnabled =
        new RobotSwitch<Boolean>(true)
            .addValue(RobotType.Prime24, true)
            .addValue(RobotType.Deux24, false)
            .get();

    public static final boolean liftSubsystemEnabled =
        new RobotSwitch<Boolean>(true)
            .addValue(RobotType.Prime24, true)
            .addValue(RobotType.Deux24, false)
            .get();
    public static final boolean algaeIntakeEnabled =
        new RobotSwitch<Boolean>(true)
            .addValue(RobotType.Deux24, false)
            .addValue(RobotType.Prime24, true)
            .get();

    public static final boolean coralOuttakeSubsystemEnabled =
        new RobotSwitch<Boolean>(true)
            .addValue(RobotType.Prime24, true)
            .addValue(RobotType.Deux24, false)
            .get();

    public static final boolean climberSubsystemEnabled =
        new RobotSwitch<Boolean>(true)
            .addValue(RobotType.Deux24, false)
            .addValue(RobotType.Prime24, true)
            .get();
    // sensors
    public static final boolean reefDetectorEnabled =
        new RobotSwitch<Boolean>(true)
            .addValue(RobotType.Prime24, true)
            .addValue(RobotType.Deux24, false)
            .get();

    // odometry
    public static final boolean gyroEnabled = new RobotSwitch<Boolean>(true).get();
  }

  public static enum PivotId {
    FL,
    FR,
    BL,
    BR;
  }

  public static class AutoAlignConfig {
    public static final double maxDistanceFromTarget = 0.3;
    public static final PathConstraints pathConstraints =
        new PathConstraints(1.5, 1, Math.PI, 4 * Math.PI);
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

    public static final double maxSpeed = 4.5;
    public static final double maxNorm =
        DriveSubsystem.computeMaxNorm(DriveConstants.positions, new Translation2d());
    public static final double maxOmega = (maxSpeed / maxNorm);
    public static final double wheelRadius = Units.inchesToMeters(2);

    public static final double accelLimit = 20;
    public static final double deaccelLimit = 11;

    public static final double initalSlope = 3.125;
    public static final double finalSlope = 4.375;

    public static final double maxSteerSpeed = 50; // rad per second

    public static final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
            frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    public static final ModuleInfo FL = new ModuleInfo(PivotId.FL, 1, 2, 2, 45);

    public static final ModuleInfo FR = new ModuleInfo(PivotId.FR, 3, 4, 0, -45);

    public static final ModuleInfo BL = new ModuleInfo(PivotId.BL, 5, 8, 1, 135);

    public static final ModuleInfo BR = new ModuleInfo(PivotId.BR, 7, 6, 3, -135);

    public static final double maxAntiTipCorrectionSpeed = 1.5;
    public static final double minTipDegrees = 3;
  }

  public static class CameraConstants {
    public static final CameraConstant frontCameraLeft =
        new CameraConstant(
            new SimCameraProperties(),
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(13.95),
                    Units.inchesToMeters(11.9),
                    Units.inchesToMeters(12.125)),
                new Rotation3d(0, Math.toRadians(10.5), -Math.toRadians(15))),
            1,
            "Sommar",
            "Front Left");

    public static final CameraConstant frontCameraRight =
        new CameraConstant(
            new SimCameraProperties(),
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(13.95),
                    -Units.inchesToMeters(11.9),
                    Units.inchesToMeters(12.125)),
                new Rotation3d(0, Math.toRadians(10.5), Math.toRadians(15))),
            1,
            "Markward",
            "Front Right");

    public static final CameraConstant backCamera =
        new CameraConstant(
            new SimCameraProperties(),
            new Transform3d(
                new Translation3d(0.146, -0.356, 0.406),
                new Rotation3d(0, Math.toRadians(1), Math.PI)),
            1,
            "BackLL",
            "Back");
    public static final Matrix<N3, N1> defaultDriveStandardDev = VecBuilder.fill(0.1, 0.1, 0.1);
    public static final Matrix<N3, N1> defaultVisionStandardDev = VecBuilder.fill(2, 2, 9999999);
  }

  public static class LiftConstants {
    public static final int liftLeaderMotorID = new RobotSwitch<Integer>(9).get();
    public static final int liftFollowerMotorID = new RobotSwitch<Integer>(10).get();
    public static final double gearRatio = 5;
    public static final Limits liftLimits = new Limits(0.0, 0.575);
    public static final double liftMaxSpeed = 2;
    public static final double liftMaxAccel = 2;
    public static final TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(liftMaxSpeed, liftMaxAccel);
    public static final double sprocketRadius = Units.inchesToMeters(1.5 / 2);
    public static final double currentThresh = 50; // for the EMA

    public static final double emaSmoothing = 10;
    public static final double emaPeriod = 21;

    public enum GantrySetpoint {
      LEFT,
      RIGHT,
      CENTER;
    }

    public enum CoralPreset {
      Pickup(0, GantrySetpoint.CENTER),
      Safe(0, 0.1, GantrySetpoint.CENTER),
      LeftL2(0.115, 0.3, GantrySetpoint.CENTER),
      RightL2(0.115, 0.3, GantrySetpoint.CENTER),
      LeftL3(0.289, 0.486, GantrySetpoint.CENTER),
      RightL3(0.289, 0.486, GantrySetpoint.CENTER),
      LeftL4(0.563, GantrySetpoint.CENTER),
      RightL4(0.563, GantrySetpoint.CENTER),
      Trough(0, GantrySetpoint.CENTER);
      /*TODO:SWITCH THIS BACK IMPORTANT!!!!!!!
      Pickup(0, GantrySetpoint.CENTER),
      Safe(0, 0.1, GantrySetpoint.CENTER),
      LeftL2(0.115, 0.3, GantrySetpoint.LEFT),
      RightL2(0.115, 0.3, GantrySetpoint.RIGHT),
      LeftL3(0.289, 0.486, GantrySetpoint.LEFT),
      RightL3(0.289, 0.486, GantrySetpoint.RIGHT),
      LeftL4(0.563, GantrySetpoint.LEFT),
      RightL4(0.563, GantrySetpoint.RIGHT),
      Trough(0, GantrySetpoint.RIGHT); */

      public final double lift;
      public final GantrySetpoint gantrySetpoint; // Driver Station side perspective
      private double liftAlgae;

      private CoralPreset(double lift, double liftAlgae, GantrySetpoint setpoint) {
        this.lift = lift;
        this.liftAlgae = liftAlgae;
        this.gantrySetpoint = setpoint;
      }

      private CoralPreset(double lift, GantrySetpoint setpoint) {
        this.lift = lift;
        this.liftAlgae = lift;
        this.gantrySetpoint = setpoint;
      }

      public double getLift() {
        return lift;
      }

      public double getLiftAlgae() {
        return liftAlgae;
      }

      public double getGantry(boolean dsSide) {
        switch (gantrySetpoint) {
          case LEFT:
            return dsSide
                ? GantryConstants.gantryLimits.low + GantryConstants.gantryPadding
                : GantryConstants.gantryLimits.high - GantryConstants.gantryPadding;
          case RIGHT:
            return !dsSide
                ? GantryConstants.gantryLimits.low + GantryConstants.gantryPadding
                : GantryConstants.gantryLimits.high - GantryConstants.gantryPadding;
          case CENTER:
            return GantryConstants.gantryLimitCenter;
          default:
            return GantryConstants.gantryLimitCenter / 2;
        }
      }

      public GantrySetpoint getGantrySetpoint(boolean dSide) {
        switch (gantrySetpoint) {
          case LEFT:
            return false ^ dSide ? GantrySetpoint.RIGHT : GantrySetpoint.LEFT;
          case RIGHT:
            return true ^ dSide ? GantrySetpoint.RIGHT : GantrySetpoint.LEFT;
          case CENTER:
            return GantrySetpoint.CENTER;
          default:
            return GantrySetpoint.CENTER;
        }
      }

      public boolean isRight() {
        return gantrySetpoint == GantrySetpoint.RIGHT;
      }
    }
  }

  public static class ReefDetectorConstants {
    public static final int channel = new RobotSwitch<Integer>(15).get();
    public static final double detectionThresh = 700;
    public static final int averageLength = 20;
    public static final double averagePercentage = 0.8;
    public static final double waitTimeSeconds = 0.1;
    public static final double timeDerivative = 0.5;
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
    public static final int climberLiftMotorID = new RobotSwitch<Integer>(15).get();
    public static final int climberWinch1MotorID = new RobotSwitch<Integer>(13).get();
    public static final int climberWinch2MotorID = new RobotSwitch<Integer>(14).get();

    public static final Limits liftLimits = new Limits(-174.874, 99999999.9);
    public static final double winchClimbedPosition = 45;
    public static final Limits winchLimits = new Limits(-99999999999999.0, 99999999999999999.0);
    public static final ResolverVoltageInfo winchResolverInfo =
        new ResolverVoltageInfo(6, 0, 5, 0, 100, null);
    public static final ResolverVoltageInfo liftResolverInfo =
        new ResolverVoltageInfo(7, 0, 5, 0, 100, null);

    public static final double gearRatio = 1;
    public static final double pulleyRadius = 1;

    public static final int solenoidForwardChannel = 2;
    public static final int solenoidReverseChannel = 3;

    public static final int sensor1Channel = 2;
    public static final int sensor2Channel = 3;
  }

  public static class GantryConstants {
    public static final int gantrySparkID = new RobotSwitch<Integer>(17).get();
    public static final double gantryGearRatio = 27.4;
    public static final double pulleyRadius = Units.inchesToMeters(0.5);
    // left -> right limit
    public static final Limits gantryLimits = new Limits(0.01, 0.36);
    public static final double gantryLimitCenter =
        (gantryLimits.high + gantryLimits.low) / 2 - 0.01;
    public static final double gantryPadding = 0.03;
    public static final int gantryLimitSwitchDIOPort = new RobotSwitch<Integer>(4).get();
    public static final double alignSpeed = 0.2;
    public static final double gantryMaxVel = 2; // PLACEHOLDER
    public static final double gantryMaxAccel = 2; // PLACEHOLDER
    public static final double gantryMaxJerk = 2; // PLACEHOLDER
    public static final TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(gantryMaxVel, gantryMaxAccel);
    public static final TrapezoidProfile.Constraints velocityConstraints =
        new TrapezoidProfile.Constraints(gantryMaxAccel, gantryMaxJerk);
  }

  public static class CoralOuttakeConstants {
    public static final double gearRatio = 0;
    public static final int intakeSparkID = new RobotSwitch<Integer>(16).get();
    // if you dont update this i will find you // *gulp* // You understand what
    // happens if you don't
    // public static final int coralDetectorChannel =
    //     new RobotSwitch<Integer>(7).get(); // update this too
    public static final int hasCoralDetectorChannel = 4; // 7 deux
    public static final double distanceRequired = 2;
    public static final double passiveSpeed = 0.25;
  }

  public static class AlgaeConstants {
    public static final int motorLeftChannel = new RobotSwitch<Integer>(11).get();
    public static final int motorRightChannel = new RobotSwitch<Integer>(12).get();
    public static final int solenoidChannelForward = 0;
    public static final int solenoidChannelReverse = 1;
    public static final double passiveSpeed = 0.1;
    public static final double highSpeed = 0.7;
    public static final double gearRatio = 1;
    public static final double currentThresh = 20; // for the EMA
    public static final double emaSmoothing = 8;
    public static final double emaPeriod = 21; // number of periods to calculate EMA over
  }
}
