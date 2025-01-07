package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.modules.ModuleInfo;

public class RobotConstants {
  public static enum PivotId {
    FL,
    FR,
    BL,
    BR;
  }

  public static class DriveConstants {
    public static double odometryFrequency = 200.0;
    public static final double driveGearRatio = 116 / 15;
    public static final double steerGearRatio = ((480 / 11)) * 1.0166667 * 0.99790377777778;
    public static final double wheelYPos = Units.inchesToMeters(22.75 / 2);
    public static final double wheelXPos = Units.inchesToMeters(22.75 / 2);
    public static final double maxSpeed = 4.6;
    public static final double wheelRadius = Units.inchesToMeters(1.8892);

    public static final double accelLimit = 20;
    public static final double deaccelLimit = 11;

    public static final double initalSlope = 1.25;
    public static final double finalSlope = 0;

    private static final Translation2d frontLeftLocation = new Translation2d(wheelXPos, wheelYPos);
    private static final Translation2d frontRightLocation =
        new Translation2d(wheelXPos, -wheelYPos);
    public static final Translation2d backLeftLocation = new Translation2d(-wheelXPos, wheelYPos);
    public static final Translation2d backRightLocation = new Translation2d(-wheelXPos, -wheelYPos);

    public static final Translation2d[] positions =
        new Translation2d[] {
          frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
        };

    public static final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
            frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    public static final ModuleInfo FL = new ModuleInfo(PivotId.FL, 3, 2, 0, 45);

    public static final ModuleInfo FR =
        new ModuleInfo(
            PivotId.FR,
            9, // 2023: and dew 1: 2
            8, // 2023: 1, dew 1: 5
            2,
            -45);

    public static final ModuleInfo BL = new ModuleInfo(PivotId.BL, 5, 4, 1, 135);

    public static final ModuleInfo BR = new ModuleInfo(PivotId.BR, 7, 6, 3, -135);
  }
}
