package frc.robot.subsystems.lift;

// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotConstants.LiftConstants;
// import frc.robot.constants.RobotPIDConstants;
import frc.robot.subsystems.lift.LiftIO.LiftIOInputs;

public class LiftIOSim implements LiftIO {
  private final DCMotorSim liftSim;
  private double liftAppliedVolts = 0.0;

  // private final PIDController liftPID =
  // RobotPIDConstants.constructPID(RobotPIDConstants.liftPID);
  // PID goes here when pid exists

  public LiftIOSim() {
    DCMotor liftGearbox = DCMotor.getNeo550(2); // motor subject to change
    liftSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(liftGearbox, 0, LiftConstants.liftGearRatio),
            liftGearbox);
  }

  @Override
  public void updateInputs(LiftIOInputs inputs) {
    liftSim.setInputVoltage(liftAppliedVolts);
    liftSim.update(0.02);

    inputs.liftConnected = true;
    inputs.liftPositionMeters = liftSim.getAngularPositionRad() * LiftConstants.wheelRadius;
    inputs.liftVelocityMetersPerSecond =
        liftSim.getAngularVelocityRadPerSec() * LiftConstants.wheelRadius;

    inputs.liftAppliedVoltage = liftAppliedVolts;
    inputs.liftCurrentAmps = liftSim.getCurrentDrawAmps();
    inputs.liftVelocities = new double[] {inputs.liftVelocityMetersPerSecond};
  }

  @Override
  public void setLiftVoltage(double voltage) {
    liftAppliedVolts = voltage;
  }

  // @Override
  // public void setLiftVelocity(double voltage, LiftIOInputs inputs) {} not entirely sure if needed

  // @Override
  // public void setLiftPosition(int pos, LiftIOInputs inputs){}
}
