package frc.robot.sensors.reefdetector;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;

public class ReefDetectorIOLaserCAN implements ReefDetectorIO {
  private LaserCan laserCan;
  private boolean isConnected;

  public ReefDetectorIOLaserCAN(int channel) {
    laserCan = new LaserCan(channel);
    isConnected = true;
    try {
      laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
      laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
      isConnected = false;
    }
  }

  public double getDistance() {
    return laserCan.getMeasurement().distance_mm;
  }

  public boolean isDetecting() {
    Measurement measurement = laserCan.getMeasurement();
    return (measurement != null
        && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);
  }

  @Override
  public void updateInputs(ReefDetectorIOInputs inputs) {
    inputs.isConnected = isConnected;
    inputs.isDetecting = isDetecting();
    inputs.distanceToReef = getDistance();
  }
}
