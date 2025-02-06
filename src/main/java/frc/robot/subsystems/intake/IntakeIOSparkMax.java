package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.constants.SparkConstants;
import frc.robot.sensors.resolvers.ResolverVoltage;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.tools.MotorLim;

public class IntakeIOSparkMax implements IntakeIO {

  private final SparkMax intakeSpark;
  private final PIDController intakePID =
      RobotPIDConstants.constructPID(RobotPIDConstants.intakePID);
  DoubleSolenoid doubleSolenoid;

  public IntakeIOSparkMax() {
    intakeSpark =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(IntakeConstants.intakeMotorID, false));
    ResolverVoltage intakeEncoder = new ResolverVoltage(IntakeConstants.intakeResolverInfo);
    doubleSolenoid =
        new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            IntakeConstants.solenoidForwardChannel,
            IntakeConstants.solenoidReverseChannel);
  }
  /*
   * Set voltage of the lift motor
   */
  @Override
  public void setIntakemotor1Voltage(double voltage, IntakeIOInputs inputs) {
    intakeSpark.setVoltage(
        MotorLim.clampVoltage(
            MotorLim.applyLimits(
                inputs.intakeMotorPosition, voltage, IntakeConstants.intakeLimits)));
  }

  /*
   * Sets the position of the lift motor using a PID
   */
  @Override
  public void setIntakemotor1Position(double position, IntakeIOInputs inputs) {
    setIntakemotor1Position(
        MotorLim.clampVoltage(intakePID.calculate(inputs.intakeMotorPosition, position)), inputs);
  }
  /*
   * Set voltage of the winch motors
   */

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeMotorCurrent = intakeSpark.getOutputCurrent();
    inputs.intakeMotorVoltage = intakeSpark.getAppliedOutput();
    inputs.intakeMotorTemperature = intakeSpark.getMotorTemperature();
  }

  @Override
  public void setIntakeSolenoidState(boolean forward, IntakeIOInputs inputs) {
    if (forward) {
      doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    } else {
      doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }
}
