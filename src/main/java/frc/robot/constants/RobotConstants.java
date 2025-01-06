package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class RobotConstants {
    public static enum PivotId {
        FL, FR, BL, BR;
    }
    public static class DriveConstants{
        public static double odometryFrequency = 200.0;
        public static final double driveGearRatio = 116 / 15;
        public static final double steerGearRatio = ((480 / 11)) * 1.0166667 * 0.99790377777778;
        public static final double wheelYPos = Units.inchesToMeters(22.75 / 2);
        public static final double wheelXPos = Units.inchesToMeters(22.75 / 2);
        public static final double maxSpeed = 4.6;

        private static final Translation2d frontLeftLocation = new Translation2d(wheelXPos, wheelYPos);
        private static final Translation2d frontRightLocation = new Translation2d(wheelXPos, -wheelYPos);
        public static final Translation2d backLeftLocation = new Translation2d(-wheelXPos, wheelYPos);
        public static final Translation2d backRightLocation = new Translation2d(-wheelXPos, -wheelYPos);

        public static final Translation2d[] positions = new Translation2d[] {
            frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation };

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
    }
}
