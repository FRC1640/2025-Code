package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO extends AutoCloseable {
    @AutoLog
    public static class ModuleIOInputs {
        public boolean driveConnected = false;
        public double drivePositionMeters;
        public double driveVelocityMetersPerSecond;
        public double driveAppliedVoltage;
        public double driveCurrentAmps;
        public double driveTempCelsius;

        public boolean turnConnected = false;
        public double steerAngleDegrees;
        public double steerRPS;
        public double steerAppliedVoltage;
        public double steerCurrentAmps;
        public double steerTempCelsius;
        public double steerAngleVoltage;
        public double steerAngleRelative;
        public int rawEncoderValue;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsMeters = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
        public double[] driveVelocities = new double[]{};
    }

    public default void updateInputs(ModuleIOInputs inputs) {
    }

    public default void setDriveVoltage(double voltage) {
    }

    public default void setDrivePercentage(double percentage) {
    }

    public default void setDriveIdleMode(boolean brake) {
    }

    public default void setSteerVoltage(double voltage) {
    }

    public default void setSteerPercentage(double percentage) {
    }
}
