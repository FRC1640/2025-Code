package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotConstants.DriveConstants;
import frc.robot.constants.RobotPIDConstants;

public class ModuleIOSim implements ModuleIO {
  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  private final PIDController drivePID = RobotPIDConstants.constructPID(RobotPIDConstants.drivePID);
  private final SimpleMotorFeedforward driveFF =
      RobotPIDConstants.constructFFSimpleMotor(RobotPIDConstants.driveFF);
  private final PIDController steerPID = RobotPIDConstants.constructPID(RobotPIDConstants.steerPID);

  public ModuleIOSim(ModuleInfo id) {
    DCMotor driveGearbox = DCMotor.getNeoVortex(1);
    DCMotor turnGearbox = DCMotor.getNeo550(1);
    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                driveGearbox, 0.00019125, DriveConstants.driveGearRatio),
            driveGearbox);
    turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                turnGearbox, 0.002174375, DriveConstants.steerGearRatio),
            turnGearbox);
  }

  @Override
  public void setDriveVelocity(double velocity, ModuleIOInputs inputs) {
    double pidSpeed = driveFF.calculate(velocity);
    pidSpeed += drivePID.calculate(inputs.driveVelocityMetersPerSecond, velocity);
    setDriveVoltage(pidSpeed);
  }

  @Override
  public void setDriveVoltage(double voltage) {
    driveAppliedVolts = voltage;
  }

  @Override
  public void setSteerPosition(Rotation2d angle, ModuleIOInputs inputs) {
    Rotation2d delta = angle.minus(Rotation2d.fromDegrees(inputs.steerAngleDegrees));
    double sin = Math.sin(delta.getRadians());
    setSteerVoltage(MathUtil.clamp(steerPID.calculate(sin, 0) * 12, -12, 12));
  }

  @Override
  public void setSteerVoltage(double voltage) {
    turnAppliedVolts = -voltage;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    driveSim.setInputVoltage(-driveAppliedVolts);
    turnSim.setInputVoltage(turnAppliedVolts);
    driveSim.update(0.02);
    turnSim.update(0.02);

    inputs.driveConnected = true;
    inputs.drivePositionMeters = driveSim.getAngularPositionRad() * DriveConstants.wheelRadius;
    inputs.driveVelocityMetersPerSecond =
        driveSim.getAngularVelocityRadPerSec() * DriveConstants.wheelRadius;

    inputs.driveAppliedVoltage = driveAppliedVolts;
    inputs.driveCurrentAmps = driveSim.getCurrentDrawAmps();

    inputs.steerConnected = true;
    inputs.steerAngleDegrees += (turnSim.getAngularVelocityRPM() * 360 / 60) * 0.02;
    inputs.steerRadPerSec = turnSim.getAngularVelocityRPM() * 2 * Math.PI / 60;
    inputs.steerAppliedVoltage = turnAppliedVolts;
    inputs.steerCurrentAmps = turnSim.getCurrentDrawAmps();

    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsMeters = new double[] {inputs.drivePositionMeters};
    inputs.odometryTurnPositions =
        new Rotation2d[] {Rotation2d.fromDegrees(inputs.steerAngleDegrees)};
    inputs.driveVelocities = new double[] {inputs.driveVelocityMetersPerSecond};
  }
}
