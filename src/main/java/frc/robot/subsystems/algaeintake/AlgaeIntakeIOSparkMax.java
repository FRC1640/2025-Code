package frc.robot.subsystems.algaeintake;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.constants.RobotConstants.AlgaeIntakeConstants;
import frc.robot.constants.SparkConstants;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.tools.MotorLim;

public class AlgaeIntakeIOSparkMax implements AlgaeIntakeIO {

  private final SparkMax intakeSpark;
  DoubleSolenoid doubleSolenoid;

  public AlgaeIntakeIOSparkMax() {
    intakeSpark =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(AlgaeIntakeConstants.algaeIntakeMotorID, false));
    doubleSolenoid =
        new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            AlgaeIntakeConstants
                .solenoidForwardChannel, // TODO: idk if we are going to use pneumatics just for
            // this.
            AlgaeIntakeConstants.solenoidReverseChannel);
  }
  /*
   * Set voltage of the lift motor
   */
  @Override
  public void setIntakeMotorVoltage(double voltage, AlgaeIntakeIOInputs inputs) {
    intakeSpark.setVoltage(
        MotorLim.clampVoltage(
            MotorLim.applyLimits(
                inputs.intakeMotorPosition,
                voltage,
                AlgaeIntakeConstants.algaeIntakeLimits.low,
                AlgaeIntakeConstants.algaeIntakeLimits.high)));
  }

  @Override
  public void updateInputs(AlgaeIntakeIOInputs inputs) {
    inputs.intakeMotorCurrent = intakeSpark.getOutputCurrent();
    inputs.intakeMotorVoltage = intakeSpark.getAppliedOutput();
    inputs.intakeMotorTemperature = intakeSpark.getMotorTemperature();
  }

  @Override
  public void setIntakeSolenoidState(boolean forward, AlgaeIntakeIOInputs inputs) {
    if (forward) {
      doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    } else {
      doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }
}
