// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.drive.ModuleIO;



public final class Main {
  private Main() {}

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
    var lowBatteryAlert = new Alert("Low Battery", AlertType.kWarning);
    double batteryVoltage;
        if (batteryVoltage < 10.5) {
      lowBatteryAlert.set(true);
    } else {
      lowBatteryAlert.set(false);
    }
  }
}

  // Update gyro alert
  gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

  private final Alert driveDisconnectedAlert;
  





  /**
   * @param io
   * @param index
   */
  public Main(ModuleIO io, int index) {
    driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
    turnDisconnectedAlert =
        new Alert(
            "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
  }

  driveDisconnectedAlert.set(!inputs.driveConnected);
  turnDisconnectedAlert.set(!inputs.turnConnected);