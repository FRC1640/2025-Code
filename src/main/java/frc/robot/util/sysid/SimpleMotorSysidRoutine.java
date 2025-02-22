package frc.robot.util.sysid;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SimpleMotorSysidRoutine {

  /**
   * Creates a new sysid routine with a single motor
   *
   * @param setVoltage consumer for motor voltage
   * @param getVoltage supplier to get motor voltage
   * @param getPositionMeters supplier for motor position
   * @param getVelocityMetersPerSecond supplier for motor velocity
   * @param subsystem subsystem for requirements
   * @param config config for sysid
   */
  public SysIdRoutine createNewRoutine(
      Consumer<Double> setVoltage,
      Supplier<Double> getVoltage,
      Supplier<Double> getPositionMeters,
      Supplier<Double> getVelocityMetersPerSecond,
      SubsystemBase subsystem,
      SysIdRoutine.Config config) {
    return new SysIdRoutine(
        config,
        new SysIdRoutine.Mechanism(
            (voltage) -> {
              setVoltage.accept(voltage.in(Volts));
            },
            (state) -> Logger.recordOutput("SysIdTestState", state.toString()),
            subsystem));
  }
}
