package frc.robot.sensors.reefdetector;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import frc.robot.constants.RobotConstants.ReefDetectorConstants;
import org.littletonrobotics.junction.Logger;

public class ReefDetectorIOLaserCAN implements ReefDetectorIO {
  private LaserCan laserCan;
  private boolean isConnected;
  private TimeInterpolatableBuffer<Double> distanceBuffer =
      TimeInterpolatableBuffer.createDoubleBuffer(2);

  public ReefDetectorIOLaserCAN() {
    laserCan = new LaserCan(ReefDetectorConstants.channel);
    isConnected = true;
    try {
      laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
      laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 2, 4, 4));
      laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_50MS);
    } catch (ConfigurationFailedException e) {
      Logger.recordOutput("LaserCAN Configuration failed! " + e.toString(), false);
      isConnected = false;
    }
  }

  public double getDistance() {
    Measurement m = laserCan.getMeasurement();
    if (m != null) {
      if (m.status == 0) {
        return m.distance_mm;
      }
    }
    return Double.MAX_VALUE;
  }

  // public boolean isDetecting() {
  //   if (getDistance() < ReefDetectorConstants.detectionThresh
  //       && distanceBuffer.getSample(System.currentTimeMillis() - 0.06).isPresent()) {
  //     if (distanceBuffer.getSample(System.currentTimeMillis() - 0.06).get() - getDistance() > 20)
  // {
  //       return true;
  //     }
  //   }

  //   return false;
  // }

  @Override
  public void updateInputs(ReefDetectorIOInputs inputs) {
    inputs.isConnected = isConnected;
    inputs.isDetecting = false;
    inputs.distanceToReef = getDistance();
    if (getDistance() < Double.MAX_VALUE) {
      distanceBuffer.addSample(System.currentTimeMillis(), getDistance());
      if (getDistance() < ReefDetectorConstants.detectionThresh) {
        if (distanceBuffer.getSample(System.currentTimeMillis() - 0.06).isPresent()) {
          if (distanceBuffer.getSample(System.currentTimeMillis() - 0.06).get() - getDistance() > 15
              && distanceBuffer.getSample(System.currentTimeMillis() - 0.06).get()
                  < ReefDetectorConstants.detectionThresh) {
            inputs.isDetecting = true;
          }
        }
      }
    }
  }
}
