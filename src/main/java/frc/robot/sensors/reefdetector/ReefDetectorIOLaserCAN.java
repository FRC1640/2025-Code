package frc.robot.sensors.reefdetector;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import frc.robot.constants.RobotConstants.ReefDetectorConstants;
import org.littletonrobotics.junction.Logger;

public class ReefDetectorIOLaserCAN implements ReefDetectorIO {
  private LaserCan laserCan;
  private boolean isConnected;

  public ReefDetectorIOLaserCAN() {
    laserCan = new LaserCan(ReefDetectorConstants.channel);
    isConnected = true;
    try {
      laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
      laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 2, 2));
      laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
    } catch (ConfigurationFailedException e) {
      Logger.recordOutput("LaserCAN Configuration failed! " + e.toString(), false);
      isConnected = false;
    }
  }

  public double getDistance() {
    Measurement m = laserCan.getMeasurement();
    if (isDetecting()) {
      return m.distance_mm;
    }
    return Double.MAX_VALUE;
  }

  public boolean isDetecting() {
    Measurement measurement = laserCan.getMeasurement();
    return (measurement != null
        && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);
  }

  @Override
  public void updateInputs(ReefDetectorIOInputs inputs) {
    inputs.isConnected = isConnected;
    inputs.isDetecting = getDistance() < ReefDetectorConstants.detectionThresh;
    inputs.distanceToReef = getDistance();
  }
}
