package frc.robot.subsystems.algae;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.RobotConstants.AlgaeConstants;
import frc.robot.constants.RobotConstants.PneumaticsConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.constants.SparkConstants;
import frc.robot.util.spark.SparkConfigurer;

public class AlgaeIOSpark implements AlgaeIO {

  private final SparkMax motorLeft;
  private final SparkMax motorRight;
  private final DoubleSolenoid solenoid;

  private final SimpleMotorFeedforward ff =
      RobotPIDConstants.constructFFSimpleMotor(RobotPIDConstants.algaeFF, "algaeFF");
  private final PIDController algaeVelocityPID =
      RobotPIDConstants.constructPID(RobotPIDConstants.algaeVelocityPID, "algaeVelocityPID");

  public AlgaeIOSpark() {
    motorLeft =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(AlgaeConstants.motorLeftChannel, false));
    motorRight =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(AlgaeConstants.motorRightChannel, true));
    solenoid =
        new DoubleSolenoid(
            PneumaticsConstants.pneumaticsHubID,
            PneumaticsModuleType.REVPH,
            AlgaeConstants.solenoidChannelForward,
            AlgaeConstants.solenoidChannelReverse);
  }

  @Override
  public void setSolenoid(boolean set) {
    solenoid.set(set ? Value.kForward : Value.kReverse);
  }

  @Override
  public void setVoltage(double left, double right) {
    motorLeft.setVoltage(left);
    motorRight.setVoltage(right);
  }

  @Override
  public void setVelocity(double leftVel, double rightVel, AlgaeIOInputs inputs) {
    setVoltage(
        (ff.calculate(leftVel)
            + algaeVelocityPID.calculate(inputs.intakeMotorLeftVelocity, leftVel)),
        (ff.calculate(rightVel)
            + algaeVelocityPID.calculate(inputs.intakeMotorRightVelocity, rightVel)));
  }

  @Override
  public void updateInputs(AlgaeIOInputs inputs) {
    inputs.intakeMotorLeftVoltage =
        motorLeft.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.intakeMotorLeftVelocity = motorLeft.getEncoder().getVelocity();
    inputs.intakeMotorLeftCurrent = motorLeft.getOutputCurrent();
    inputs.intakeMotorLeftTemperature = motorLeft.getMotorTemperature();

    inputs.intakeMotorRightVoltage =
        motorRight.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.intakeMotorRightVelocity = motorRight.getEncoder().getVelocity();
    inputs.intakeMotorRightCurrent = motorRight.getOutputCurrent();
    inputs.intakeMotorRightTemperature = motorRight.getMotorTemperature();

    inputs.solenoidForward = solenoid.get() == Value.kForward;
  }
}
