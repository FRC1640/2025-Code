package frc.robot.subsystems.algaeintake;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.constants.RobotConstants.IntakeConstants;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.constants.SparkConstants;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.tools.MotorLim;

public class AlgaeIntakeIOSparkMax implements AlgaeIntakeIO {

  private final SparkMax intakeSpark;
  private final PIDController intakePID =
      RobotPIDConstants.constructPID(RobotPIDConstants.intakePID);
  DoubleSolenoid doubleSolenoid;

  public AlgaeIntakeIOSparkMax() {
    intakeSpark =
        SparkConfigurer.configSparkMax(
            SparkConstants.getDefaultMax(IntakeConstants.intakeMotorID, false));
    doubleSolenoid =
        new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            IntakeConstants
                .solenoidForwardChannel, // TODO: idk if we are going to use pneumatics just for
            // this.
            IntakeConstants.solenoidReverseChannel);
  }
  /*
   * Set voltage of the lift motor
   */
  @Override
  public void setIntakeMotorVoltage(double voltage, AlgaeIntakeIOInputs inputs) {
    intakeSpark.setVoltage(
        MotorLim.clampVoltage(
            MotorLim.applyLimits(
                inputs.intakeMotorPosition, voltage, IntakeConstants.intakeLimits)));
  }

  /*
   * Sets the position of the lift motor using a PID
   */
  @Override
  public void setIntakeMotorPosition(double position, AlgaeIntakeIOInputs inputs) {
    setIntakeMotorPosition(
        MotorLim.clampVoltage(intakePID.calculate(inputs.intakeMotorPosition, position)), inputs);
  }
  /*
   * Set voltage of the winch motors
   */

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
