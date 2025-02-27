package frc.robot.sensors.gyro.imported;

import com.studica.frc.AHRS.BoardYawAxis;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;
import com.studica.frc.jni.AHRSJNI;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

/**
 * The AHRS class provides an interface to AHRS capabilities of the KauaiLabs navX Robotics
 * Navigation Sensor via SPI, I2C and Serial (TTL UART and USB) communications interfaces on the
 * RoboRIO.
 *
 * <p>The AHRS class enables access to basic connectivity and state information, as well as key
 * 6-axis and 9-axis orientation information (yaw, pitch, roll, compass heading, fused (9-axis)
 * heading and magnetic disturbance detection.
 *
 * <p>Additionally, the ARHS class also provides access to extended information including linear
 * acceleration, motion detection, rotation detection and sensor temperature.
 *
 * <p>If used with the navX Aero, the AHRS class also provides access to altitude, barometric
 * pressure and pressure sensor temperature data
 */
public class AHRS implements NTSendable, AutoCloseable {
  /*
   * Port # for sim
   */
  private int port = 0;

  /**
   * Identifies one of the three sensing axes on the navX sensor board. Note that these axes are
   * board-relative ("Board Frame"), and are not necessarily the same as the logical axes of the
   * chassis on which the sensor is mounted.
   *
   * <p>For more information on sensor orientation, please see the navX sensor <a
   * href=http://navx-mxp.kauailabs.com/installation/orientation-2/>Orientation</a> page.
   */
  public enum BoardAxis {
    kBoardAxisX(0),
    kBoardAxisY(1),
    kBoardAxisZ(2);

    private int value;

    private BoardAxis(int value) {
      this.value = value;
    }

    public int getValue() {
      return this.value;
    }
  };

  /**
   * Constructs the AHRS class using the selected communication. The update rate will be the
   * default.
   *
   * @param comType Com Port to use
   */
  public AHRS(NavXComType comType) {
    AHRSJNI.c_AHRS_create(comType);
    switch (comType.getValue()) {
      case 0: // MXP SPI
        SendableRegistry.addLW(this, "navX-Sensor", 4);
        port = 4;
        break;
      case 1: // MXP UART
        SendableRegistry.addLW(this, "navX-Sensor", 1);
        port = 1;
        break;
      case 2: // kUSB1
        SendableRegistry.addLW(this, "navX-Sensor", 2);
        port = 2;
        break;
      case 3: // kUSB2
        SendableRegistry.addLW(this, "navX-Sensor", 3);
        port = 3;
        break;
      case 4: // MXP I2C
        SendableRegistry.addLW(this, "navX-Sensor", 1);
        port = 1;
        break;
      default:
        break;
    }
  }

  /**
   * Constructs the AHRS class using the selected communication, overriding the default update rate
   * with a custom rate which may be from 4 to 200, representing the number of updates per second
   * sent by the sensor. Note that increasing the update rate may increase the CPU utilization.
   *
   * <p>This constructor provides an enum for setting the update rate. These are preselected rates
   * to ensure stable update rates. For update rates outside the enum use the {@link
   * #AHRS(NavXComType comType, int customRateHz)} constructor;
   *
   * @param comType Com Port to use
   * @param updateRate Custom Update Rate (Hz)
   */
  public AHRS(NavXComType comType, NavXUpdateRate updateRate) {
    AHRSJNI.c_AHRS_create_rate(comType, updateRate);
    switch (comType.getValue()) {
      case 0: // MXP SPI
        SendableRegistry.addLW(this, "navX-Sensor", 4);
        port = 4;
        break;
      case 1: // MXP UART
        SendableRegistry.addLW(this, "navX-Sensor", 1);
        port = 1;
        break;
      case 2: // kUSB1
        SendableRegistry.addLW(this, "navX-Sensor", 2);
        port = 2;
        break;
      case 3: // kUSB2
        SendableRegistry.addLW(this, "navX-Sensor", 3);
        port = 3;
        break;
      case 4: // MXP I2C
        SendableRegistry.addLW(this, "navX-Sensor", 1);
        port = 1;
        break;
      default:
        break;
    }
  }

  /**
   * Constructs the AHRS class using the selected communication, overriding the default update rate
   * with a custom rate which may be from 4 to 200, representing the number of updates per second
   * sent by the sensor. Note that increasing the update rate may increase the CPU utilization.
   *
   * <p>The actual sample rate is rounded down to the nearest integer that is divisible by the
   * number of Digital Motion Processor clock ticks. For instance, a request for 58 Hertz will
   * result in an actual rate of 66Hz (200 / (200 / 58), using integer math.
   *
   * @param comType Com Port to use
   * @param customRateHz Custom Update Rate (Hz)
   */
  public AHRS(NavXComType comType, int customRateHz) {
    if (customRateHz < 4 || customRateHz > 200) {
      System.out.println("Navx: Custom Rate in AHRS constructor set out of bounds.");
      return;
    } else {
      AHRSJNI.c_AHRS_create_custom(comType, customRateHz);
      switch (comType.getValue()) {
        case 0: // MXP SPI
          SendableRegistry.addLW(this, "navX-Sensor", 4);
          break;
        case 1: // MXP UART
          SendableRegistry.addLW(this, "navX-Sensor", 1);
          break;
        case 2: // kUSB1
          SendableRegistry.addLW(this, "navX-Sensor", 2);
          break;
        case 3: // kUSB2
          SendableRegistry.addLW(this, "navX-Sensor", 3);
          break;
        case 4: // MXP I2C
          SendableRegistry.addLW(this, "navX-Sensor", 1);
          break;
        default:
          break;
      }
    }
  }

  /** Destucts the AHRS C++ Instance */
  private void destroy() {
    AHRSJNI.c_AHRS_destroy();
  }

  /**
   * Returns the port number of the navx.
   *
   * @return port number
   */
  public int getPort() {
    return port;
  }

  /**
   * Returns the current pitch value (in degrees, from -180 to 180) reported by the sensor. Pitch is
   * a measure of rotation around the X Axis.
   *
   * @return The current pitch value in degrees (-180 to 180).
   */
  public float getPitch() {
    return AHRSJNI.c_AHRS_GetPitch();
  }

  /**
   * Returns the current roll value (in degrees, from -180 to 180) reported by the sensor. Roll is a
   * measure of rotation around the X Axis.
   *
   * @return The current roll value in degrees (-180 to 180).
   */
  public float getRoll() {
    return AHRSJNI.c_AHRS_GetRoll();
  }

  /**
   * Returns the current yaw value (in degrees, from -180 to 180) reported by the sensor. Yaw is a
   * measure of rotation around the Z Axis (which is perpendicular to the earth).
   *
   * <p>Note that the returned yaw value will be offset by a user-specified offset value; this
   * user-specified offset value is set by invoking the zeroYaw() method.
   *
   * @return The current yaw value in degrees (-180 to 180).
   */
  public float getYaw() {
    return AHRSJNI.c_AHRS_GetYaw();
  }

  /**
   * Returns the current tilt-compensated compass heading value (in degrees, from 0 to 360) reported
   * by the sensor.
   *
   * <p>Note that this value is sensed by a magnetometer, which can be affected by nearby magnetic
   * fields (e.g., the magnetic fields generated by nearby motors).
   *
   * <p>Before using this value, ensure that (a) the magnetometer has been calibrated and (b) that a
   * magnetic disturbance is not taking place at the instant when the compass heading was generated.
   *
   * @return The current tilt-compensated compass heading, in degrees (0-360).
   */
  public float getCompassHeading() {
    return AHRSJNI.c_AHRS_GetCompassHeading();
  }

  /**
   * Sets the user-specified yaw offset to the current yaw value reported by the sensor.
   *
   * <p>This user-specified yaw offset is automatically subtracted from subsequent yaw values
   * reported by the getYaw() method.
   *
   * <p>NOTE: This method has no effect if the sensor is currently calibrating, since resetting the
   * yaw will interfere with the calibration process.
   */
  public void zeroYaw() {
    AHRSJNI.c_AHRS_ZeroYaw();
  }

  /**
   * Return the heading of the robot as a {@link edu.wpi.first.math.geometry.Rotation2d}.
   *
   * <p>The angle is continuous, that is it will continue from 360 to 361 degrees. This allows
   * algorithms that wouldn't want to see a discontinuity in the gyro output as it sweeps past from
   * 360 to 0 on the second time around.
   *
   * <p>The angle is expected to increase as the gyro turns counterclockwise when looked at from the
   * top. It needs to follow the NWU axis convention.
   *
   * <p>This heading is based on integration of the returned rate from the gyro.
   *
   * @return the current heading of the robot as a {@link edu.wpi.first.math.geometry.Rotation2d}.
   */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(-AHRSJNI.c_AHRS_GetAngle());
  }

  /** Constructs a Rotation3d from a quaternion. */
  /**
   * Constructs a Rotation3d from a quaternion
   *
   * @return the Rotation3d
   */
  public Rotation3d getRotation3d() {
    Quaternion q =
        new Quaternion(
            AHRSJNI.c_AHRS_GetQuaternionW(),
            AHRSJNI.c_AHRS_GetQuaternionX(),
            AHRSJNI.c_AHRS_GetQuaternionY(),
            AHRSJNI.c_AHRS_GetQuaternionZ());
    return new Rotation3d(q);
  }

  /**
   * Returns true if the sensor is currently performing automatic gyro/accelerometer calibration.
   * Automatic calibration occurs when the sensor is initially powered on, during which time the
   * sensor should be held still, with the Z-axis pointing up (perpendicular to the earth).
   *
   * <p>NOTE: During this automatic calibration, the yaw, pitch and roll values returned may not be
   * accurate.
   *
   * <p>Once calibration is complete, the sensor will automatically remove an internal yaw offset
   * value from all reported values.
   *
   * <p>
   *
   * @return Returns true if the sensor is currently automatically calibrating the gyro and
   *     accelerometer sensors.
   */
  public boolean isCalibrating() {
    return AHRSJNI.c_AHRS_IsCalibrating();
  }

  /**
   * Indicates whether the sensor is currently connected to the host computer. A connection is
   * considered established whenever communication with the sensor has occurred recently.
   *
   * <p>
   *
   * @return Returns true if a valid update has been recently received from the sensor.
   */
  public boolean isConnected() {
    return AHRSJNI.c_AHRS_IsConnected();
  }

  /**
   * Returns the count in bytes of data received from the sensor. This could can be useful for
   * diagnosing connectivity issues.
   *
   * <p>If the byte count is increasing, but the update count (see getUpdateCount()) is not, this
   * indicates a software misconfiguration.
   *
   * @return The number of bytes received from the sensor.
   */
  public double getByteCount() {
    return AHRSJNI.c_AHRS_GetByteCount();
  }

  /**
   * Returns the count of valid updates which have been received from the sensor. This count should
   * increase at the same rate indicated by the configured update rate.
   *
   * @return The number of valid updates received from the sensor.
   */
  public double getUpdateCount() {
    return AHRSJNI.c_AHRS_GetUpdateCount();
  }

  /**
   * Returns the sensor timestamp corresponding to the last sample retrieved from the sensor. Note
   * that this sensor timestamp is only provided when the Register-based IO methods (SPI, I2C) are
   * used; sensor timestamps are not provided when Serial-based IO methods (TTL UART, USB) are used.
   *
   * @return The sensor timestamp corresponding to the current AHRS sensor data.
   */
  public long getLastSensorTimestamp() {
    return AHRSJNI.c_AHRS_GetLastSensorTimestamp();
  }

  /**
   * Returns the current linear acceleration in the X-axis (in G).
   *
   * <p>World linear acceleration refers to raw acceleration data, which has had the gravity
   * component removed, and which has been rotated to the same reference frame as the current yaw
   * value. The resulting value represents the current acceleration in the x-axis of the body (e.g.,
   * the robot) on which the sensor is mounted.
   *
   * <p>
   *
   * @return Current world linear acceleration in the X-axis (in G).
   */
  public float getWorldLinearAccelX() {
    return AHRSJNI.c_AHRS_GetWorldLinearAccelX();
  }

  /**
   * Returns the current linear acceleration in the Y-axis (in G).
   *
   * <p>World linear acceleration refers to raw acceleration data, which has had the gravity
   * component removed, and which has been rotated to the same reference frame as the current yaw
   * value. The resulting value represents the current acceleration in the Y-axis of the body (e.g.,
   * the robot) on which the sensor is mounted.
   *
   * <p>
   *
   * @return Current world linear acceleration in the Y-axis (in G).
   */
  public float getWorldLinearAccelY() {
    return AHRSJNI.c_AHRS_GetWorldLinearAccelY();
  }

  /**
   * Returns the current linear acceleration in the Z-axis (in G).
   *
   * <p>World linear acceleration refers to raw acceleration data, which has had the gravity
   * component removed, and which has been rotated to the same reference frame as the current yaw
   * value. The resulting value represents the current acceleration in the Z-axis of the body (e.g.,
   * the robot) on which the sensor is mounted.
   *
   * <p>
   *
   * @return Current world linear acceleration in the Z-axis (in G).
   */
  public float getWorldLinearAccelZ() {
    return AHRSJNI.c_AHRS_GetWorldLinearAccelZ();
  }

  /**
   * Indicates if the sensor is currently detecting motion, based upon the X and Y-axis world linear
   * acceleration values. If the sum of the absolute values of the X and Y axis exceed a "motion
   * threshold", the motion state is indicated.
   *
   * <p>
   *
   * @return Returns true if the sensor is currently detecting motion.
   */
  public boolean isMoving() {
    return AHRSJNI.c_AHRS_IsMoving();
  }

  /**
   * Indicates if the sensor is currently detecting yaw rotation, based upon whether the change in
   * yaw over the last second exceeds the "Rotation Threshold."
   *
   * <p>Yaw Rotation can occur either when the sensor is rotating, or when the sensor is not
   * rotating AND the current gyro calibration is insufficiently calibrated to yield the standard
   * yaw drift rate.
   *
   * <p>
   *
   * @return Returns true if the sensor is currently detecting motion.
   */
  public boolean isRotating() {
    return AHRSJNI.c_AHRS_IsRotating();
  }

  /**
   * Returns the current barometric pressure, based upon calibrated readings from the onboard
   * pressure sensor. This value is in units of millibar.
   *
   * <p>NOTE: This value is only valid for a navX Aero. To determine whether this value is valid,
   * see isAltitudeValid().
   *
   * @return Returns current barometric pressure (navX Aero only).
   */
  public float getBarometricPressure() {
    return AHRSJNI.c_AHRS_GetBarometricPressure();
  }

  /**
   * Returns the current altitude, based upon calibrated readings from a barometric pressure sensor,
   * and the currently-configured sea-level barometric pressure [navX Aero only]. This value is in
   * units of meters.
   *
   * <p>NOTE: This value is only valid sensors including a pressure sensor. To determine whether
   * this value is valid, see isAltitudeValid().
   *
   * <p>
   *
   * @return Returns current altitude in meters (as long as the sensor includes an installed
   *     on-board pressure sensor).
   */
  public float getAltitude() {
    return AHRSJNI.c_AHRS_GetAltitude();
  }

  /**
   * Indicates whether the current altitude (and barometric pressure) data is valid. This value will
   * only be true for a sensor with an onboard pressure sensor installed.
   *
   * <p>If this value is false for a board with an installed pressure sensor, this indicates a
   * malfunction of the onboard pressure sensor.
   *
   * <p>
   *
   * @return Returns true if a working pressure sensor is installed.
   */
  public boolean isAltitudeValid() {
    return AHRSJNI.c_AHRS_IsAltitudeValid();
  }

  /**
   * Returns the "fused" (9-axis) heading.
   *
   * <p>The 9-axis heading is the fusion of the yaw angle, the tilt-corrected compass heading, and
   * magnetic disturbance detection. Note that the magnetometer calibration procedure is required in
   * order to achieve valid 9-axis headings.
   *
   * <p>The 9-axis Heading represents the sensor's best estimate of current heading, based upon the
   * last known valid Compass Angle, and updated by the change in the Yaw Angle since the last known
   * valid Compass Angle. The last known valid Compass Angle is updated whenever a Calibrated
   * Compass Angle is read and the sensor has recently rotated less than the Compass Noise Bandwidth
   * (~2 degrees).
   *
   * @return Fused Heading in Degrees (range 0-360)
   */
  public float getFusedHeading() {
    return AHRSJNI.c_AHRS_GetFusedHeading();
  }

  /**
   * Indicates whether the current magnetic field strength diverges from the calibrated value for
   * the earth's magnetic field by more than the currently- configured Magnetic Disturbance Ratio.
   *
   * <p>This function will always return false if the sensor's magnetometer has not yet been
   * calibrated; see isMagnetometerCalibrated().
   *
   * @return true if a magnetic disturbance is detected (or the magnetometer is uncalibrated).
   */
  public boolean isMagneticDisturbance() {
    return AHRSJNI.c_AHRS_IsMagneticDisturbance();
  }

  /**
   * Indicates whether the magnetometer has been calibrated.
   *
   * <p>Magnetometer Calibration must be performed by the user.
   *
   * <p>Note that if this function does indicate the magnetometer is calibrated, this does not
   * necessarily mean that the calibration quality is sufficient to yield valid compass headings.
   *
   * <p>
   *
   * @return Returns true if magnetometer calibration has been performed.
   */
  public boolean isMagnetometerCalibrated() {
    return AHRSJNI.c_AHRS_IsMagnetometerCalibrated();
  }

  /**
   * Returns the imaginary portion (W) of the Orientation Quaternion which fully describes the
   * current sensor orientation with respect to the reference angle defined as the angle at which
   * the yaw was last "zeroed".
   *
   * <p>Each quaternion value (W,X,Y,Z) is expressed as a value ranging from -1 to 1. This total
   * range (2) can be associated with a unit circle, since each circle is comprised of 2 PI Radians.
   *
   * <p>For more information on Quaternions and their use, please see this <a
   * href=https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation>definition</a>.
   *
   * @return Returns the imaginary portion (W) of the quaternion.
   */
  public float getQuaternionW() {
    return AHRSJNI.c_AHRS_GetQuaternionW();
  }

  /**
   * Returns the real portion (X axis) of the Orientation Quaternion which fully describes the
   * current sensor orientation with respect to the reference angle defined as the angle at which
   * the yaw was last "zeroed".
   *
   * <p>Each quaternion value (W,X,Y,Z) is expressed as a value ranging from -1 to 1. This total
   * range (2) can be associated with a unit circle, since each circle is comprised of 2 PI Radians.
   *
   * <p>For more information on Quaternions and their use, please see this <a
   * href=https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation>description</a>.
   *
   * @return Returns the real portion (X) of the quaternion.
   */
  public float getQuaternionX() {
    return AHRSJNI.c_AHRS_GetQuaternionX();
  }

  /**
   * Returns the real portion (Y axis) of the Orientation Quaternion which fully describes the
   * current sensor orientation with respect to the reference angle defined as the angle at which
   * the yaw was last "zeroed".
   *
   * <p>Each quaternion value (W,X,Y,Z) is expressed as a value ranging from -1 to 1. This total
   * range (2) can be associated with a unit circle, since each circle is comprised of 2 PI Radians.
   *
   * <p>For more information on Quaternions and their use, please see:
   *
   * <p>https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
   *
   * @return Returns the real portion (Y) of the quaternion.
   */
  public float getQuaternionY() {
    return AHRSJNI.c_AHRS_GetQuaternionY();
  }

  /**
   * Returns the real portion (Z axis) of the Orientation Quaternion which fully describes the
   * current sensor orientation with respect to the reference angle defined as the angle at which
   * the yaw was last "zeroed".
   *
   * <p>Each quaternion value (W,X,Y,Z) is expressed as a value ranging from -1 to 1. This total
   * range (2) can be associated with a unit circle, since each circle is comprised of 2 PI Radians.
   *
   * <p>For more information on Quaternions and their use, please see:
   *
   * <p>https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
   *
   * @return Returns the real portion (Z) of the quaternion.
   */
  public float getQuaternionZ() {
    return AHRSJNI.c_AHRS_GetQuaternionZ();
  }

  /**
   * Zeros the displacement integration variables. Invoke this at the moment when integration
   * begins.
   */
  public void resetDisplacement() {
    AHRSJNI.c_AHRS_ResetDisplacement();
  }

  /**
   * Each time new linear acceleration samples are received, this function should be invoked. This
   * function transforms acceleration in G to meters/sec^2, then converts this value to Velocity in
   * meters/sec (based upon velocity in the previous sample). Finally, this value is converted to
   * displacement in meters, and integrated.
   *
   * @param accelXG
   * @param accelYG
   * @param updateRateHz
   * @param isMoving
   */
  public void updateDisplacement(float accelXG, float accelYG, int updateRateHz, boolean isMoving) {
    AHRSJNI.c_AHRS_UpdateDisplacement(accelXG, accelYG, updateRateHz, isMoving);
  }

  /**
   * Call this to configure swapable axes for X/Y or to invert an axis. Currently, this will also
   * swap/invert the robot centic values.
   *
   * @param swapAxes Will swap X/Y Axis
   * @param invertX Will invert X
   * @param invertY Will invert Y
   * @param invertZ Will invert Z
   */
  public void configureVelocity(
      boolean swapAxes, boolean invertX, boolean invertY, boolean invertZ) {
    AHRSJNI.c_AHRS_ConfigureVelocity(swapAxes, invertX, invertY, invertZ);
  }

  /**
   * Returns the velocity (in meters/sec) of the X axis [Experimental].
   *
   * <p>NOTE: This feature is experimental. Velocity measures rely on integration of acceleration
   * values from MEMS accelerometers which yield "noisy" values. The resulting velocities are not
   * known to be very accurate.
   *
   * @return Current Velocity (in meters/sec).
   */
  public float getVelocityX() {
    return AHRSJNI.c_AHRS_GetVelocityX();
  }

  /**
   * Returns the velocity (in meters/sec) of the Y axis [Experimental].
   *
   * <p>NOTE: This feature is experimental. Velocity measures rely on integration of acceleration
   * values from MEMS accelerometers which yield "noisy" values. The resulting velocities are not
   * known to be very accurate.
   *
   * @return Current Velocity (in meters/sec).
   */
  public float getVelocityY() {
    return AHRSJNI.c_AHRS_GetVelocityY();
  }

  /**
   * Returns the velocity (in meters/sec) of the Z axis [Experimental].
   *
   * <p>NOTE: This feature is experimental. Velocity measures rely on integration of acceleration
   * values from MEMS accelerometers which yield "noisy" values. The resulting velocities are not
   * known to be very accurate.
   *
   * @return Current Velocity (in meters/sec).
   */
  public float getVelocityZ() {
    return AHRSJNI.c_AHRS_GetVelocityZ();
  }

  /**
   * Returns the displacement (in meters) of the X axis since resetDisplacement() was last invoked
   * [Experimental].
   *
   * <p>NOTE: This feature is experimental. Displacement measures rely on double-integration of
   * acceleration values from MEMS accelerometers which yield "noisy" values. The resulting
   * displacement are not known to be very accurate, and the amount of error increases quickly as
   * time progresses.
   *
   * @return Displacement since last reset (in meters).
   */
  public float getDisplacementX() {
    return AHRSJNI.c_AHRS_GetDisplacementX();
  }

  /**
   * Returns the displacement (in meters) of the Y axis since resetDisplacement() was last invoked
   * [Experimental].
   *
   * <p>NOTE: This feature is experimental. Displacement measures rely on double-integration of
   * acceleration values from MEMS accelerometers which yield "noisy" values. The resulting
   * displacement are not known to be very accurate, and the amount of error increases quickly as
   * time progresses.
   *
   * @return Displacement since last reset (in meters).
   */
  public float getDisplacementY() {
    return AHRSJNI.c_AHRS_GetDisplacementY();
  }

  /**
   * Returns the displacement (in meters) of the Z axis since resetDisplacement() was last invoked
   * [Experimental].
   *
   * <p>NOTE: This feature is experimental. Displacement measures rely on double-integration of
   * acceleration values from MEMS accelerometers which yield "noisy" values. The resulting
   * displacement are not known to be very accurate, and the amount of error increases quickly as
   * time progresses.
   *
   * @return Displacement since last reset (in meters).
   */
  public float getDisplacementZ() {
    return AHRSJNI.c_AHRS_GetDisplacementZ();
  }

  /**
   * Returns the velocity (in meters/sec) of the Robot Centric X axis [Experimental].
   *
   * <p>NOTE: This feature is experimental. Velocity measures rely on integration of acceleration
   * values from MEMS accelerometers which yield "noisy" values. The resulting velocities are not
   * known to be very accurate. Uses Quaternions as heading for better accuracy and avoids gimbal
   * lock.
   *
   * @return Current Velocity (in meters/sec).
   */
  public float getRobotCentricVelocityX() {
    return AHRSJNI.c_AHRS_GetRobotCentricVelocityX();
  }

  /**
   * Returns the velocity (in meters/sec) of the Robot Centric Y axis [Experimental].
   *
   * <p>NOTE: This feature is experimental. Velocity measures rely on integration of acceleration
   * values from MEMS accelerometers which yield "noisy" values. The resulting velocities are not
   * known to be very accurate. Uses Quaternions as heading for better accuracy and avoids gimbal
   * lock.
   *
   * @return Current Velocity (in meters/sec).
   */
  public float getRobotCentricVelocityY() {
    return AHRSJNI.c_AHRS_GetRobotCentricVelocityX();
  }

  /**
   * Returns the velocity (in meters/sec) of the Robot Centric Z axis [Experimental].
   *
   * <p>NOTE: This feature is experimental. Velocity measures rely on integration of acceleration
   * values from MEMS accelerometers which yield "noisy" values. The resulting velocities are not
   * known to be very accurate. Uses Quaternions as heading for better accuracy and avoids gimbal
   * lock.
   *
   * @return Current Velocity (in meters/sec).
   */
  public float getRobotCentricVelocityZ() {
    return AHRSJNI.c_AHRS_GetRobotCentricVelocityX();
  }

  /**
   * Returns the total accumulated yaw angle (Z Axis, in degrees) reported by the sensor.
   *
   * <p>NOTE: The angle is continuous, meaning it's range is beyond 360 degrees. This ensures that
   * algorithms that wouldn't want to see a discontinuity in the gyro output as it sweeps past 0 on
   * the second time around.
   *
   * <p>Note that the returned yaw value will be offset by a user-specified offset value; this
   * user-specified offset value is set by invoking the zeroYaw() method.
   *
   * <p>
   *
   * @return The current total accumulated yaw angle (Z axis) of the robot in degrees. This heading
   *     is based on integration of the returned rate from the Z-axis (yaw) gyro.
   */
  public double getAngle() {
    return AHRSJNI.c_AHRS_GetAngle();
  }

  /**
   * Return the rate of rotation of the yaw (Z-axis) gyro, in degrees per second.
   *
   * <p>The rate is based on the most recent reading of the yaw gyro angle.
   *
   * <p>
   *
   * @return The current rate of change in yaw angle (in degrees per second)
   */
  public double getRate() {
    return AHRSJNI.c_AHRS_GetRate();
  }

  /**
   * Sets an amount of angle to be automatically added before returning a angle from the getAngle()
   * method. This allows users of the getAngle() method to logically rotate the sensor by a given
   * amount of degrees.
   *
   * <p>NOTE 1: The adjustment angle is <b>only</b> applied to the value returned from getAngle() -
   * it does not adjust the value returned from getYaw(), nor any of the quaternion values.
   *
   * <p>NOTE 2: The adjustment angle is <b>not</b>automatically cleared whenever the sensor yaw
   * angle is reset.
   *
   * <p>If not set, the default adjustment angle is 0 degrees (no adjustment).
   *
   * @param angle, in degrees (range: -360 to 360)
   */
  public void setAngleAdjustment(double angle) {
    AHRSJNI.c_AHRS_SetAngleAdjustment(angle);
  }

  /**
   * Returns the currently configured adjustment angle. See setAngleAdjustment() for more details.
   *
   * <p>If this method returns 0 degrees, no adjustment to the value returned via getAngle() will
   * occur.
   *
   * @return adjustment, in degrees (range: -360 to 360)
   */
  public double getAngleAdjustment() {
    return AHRSJNI.c_AHRS_GetAngleAdjustment();
  }

  /**
   * Reset the Yaw gyro.
   *
   * <p>Resets the Gyro Z (Yaw) axis to a heading of zero. This can be used if there is significant
   * drift in the gyro and it needs to be recalibrated after it has been running.
   */
  public void reset() {
    AHRSJNI.c_AHRS_Reset();
  }

  /**
   * Returns the current raw (unprocessed) X-axis gyro rotation rate (in degrees/sec). NOTE: this
   * value is un-processed, and should only be accessed by advanced users. Typically, rotation about
   * the X Axis is referred to as "Pitch". Calibrated and Integrated Pitch data is accessible via
   * the {@link #getPitch()} method.
   *
   * <p>
   *
   * @return Returns the current rotation rate (in degrees/sec).
   */
  public float getRawGyroX() {
    return AHRSJNI.c_AHRS_GetRawGyroX();
  }

  /**
   * Returns the current raw (unprocessed) Y-axis gyro rotation rate (in degrees/sec). NOTE: this
   * value is un-processed, and should only be accessed by advanced users. Typically, rotation about
   * the T Axis is referred to as "Roll". Calibrated and Integrated Pitch data is accessible via the
   * {@link #getRoll()} method.
   *
   * <p>
   *
   * @return Returns the current rotation rate (in degrees/sec).
   */
  public float getRawGyroY() {
    return AHRSJNI.c_AHRS_GetRawGyroY();
  }

  /**
   * Returns the current raw (unprocessed) Z-axis gyro rotation rate (in degrees/sec). NOTE: this
   * value is un-processed, and should only be accessed by advanced users. Typically, rotation about
   * the T Axis is referred to as "Yaw". Calibrated and Integrated Pitch data is accessible via the
   * {@link #getYaw()} method.
   *
   * <p>
   *
   * @return Returns the current rotation rate (in degrees/sec).
   */
  public float getRawGyroZ() {
    return AHRSJNI.c_AHRS_GetRawGyroZ();
  }

  /**
   * Returns the current raw (unprocessed) X-axis acceleration rate (in G). NOTE: this value is
   * unprocessed, and should only be accessed by advanced users. This raw value has not had
   * acceleration due to gravity removed from it, and has not been rotated to the world reference
   * frame. Gravity-corrected, world reference frame-corrected X axis acceleration data is
   * accessible via the {@link #getWorldLinearAccelX()} method.
   *
   * <p>
   *
   * @return Returns the current acceleration rate (in G).
   */
  public float getRawAccelX() {
    return AHRSJNI.c_AHRS_GetRawAccelX();
  }

  /**
   * Returns the current raw (unprocessed) Y-axis acceleration rate (in G). NOTE: this value is
   * unprocessed, and should only be accessed by advanced users. This raw value has not had
   * acceleration due to gravity removed from it, and has not been rotated to the world reference
   * frame. Gravity-corrected, world reference frame-corrected Y axis acceleration data is
   * accessible via the {@link #getWorldLinearAccelY()} method.
   *
   * <p>
   *
   * @return Returns the current acceleration rate (in G).
   */
  public float getRawAccelY() {
    return AHRSJNI.c_AHRS_GetRawAccelY();
  }

  /**
   * Returns the current raw (unprocessed) Z-axis acceleration rate (in G). NOTE: this value is
   * unprocessed, and should only be accessed by advanced users. This raw value has not had
   * acceleration due to gravity removed from it, and has not been rotated to the world reference
   * frame. Gravity-corrected, world reference frame-corrected Z axis acceleration data is
   * accessible via the {@link #getWorldLinearAccelZ()} method.
   *
   * <p>
   *
   * @return Returns the current acceleration rate (in G).
   */
  public float getRawAccelZ() {
    return AHRSJNI.c_AHRS_GetRawAccelZ();
  }

  /**
   * Returns the current raw (unprocessed) X-axis magnetometer reading (in uTesla). NOTE: this value
   * is unprocessed, and should only be accessed by advanced users. This raw value has not been
   * tilt-corrected, and has not been combined with the other magnetometer axis data to yield a
   * compass heading. Tilt-corrected compass heading data is accessible via the {@link
   * #getCompassHeading()} method.
   *
   * <p>
   *
   * @return Returns the mag field strength (in uTesla).
   */
  public float getRawMagX() {
    return AHRSJNI.c_AHRS_GetRawMagX();
  }

  /**
   * Returns the current raw (unprocessed) Y-axis magnetometer reading (in uTesla). NOTE: this value
   * is unprocessed, and should only be accessed by advanced users. This raw value has not been
   * tilt-corrected, and has not been combined with the other magnetometer axis data to yield a
   * compass heading. Tilt-corrected compass heading data is accessible via the {@link
   * #getCompassHeading()} method.
   *
   * <p>
   *
   * @return Returns the mag field strength (in uTesla).
   */
  public float getRawMagY() {
    return AHRSJNI.c_AHRS_GetRawMagY();
  }

  /**
   * Returns the current raw (unprocessed) Z-axis magnetometer reading (in uTesla). NOTE: this value
   * is unprocessed, and should only be accessed by advanced users. This raw value has not been
   * tilt-corrected, and has not been combined with the other magnetometer axis data to yield a
   * compass heading. Tilt-corrected compass heading data is accessible via the {@link
   * #getCompassHeading()} method.
   *
   * <p>
   *
   * @return Returns the mag field strength (in uTesla).
   */
  public float getRawMagZ() {
    return AHRSJNI.c_AHRS_GetRawMagZ();
  }

  /**
   * Returns the current barometric pressure (in millibar) [navX Aero only].
   *
   * <p>This value is valid only if a barometric pressure sensor is onboard.
   *
   * @return Returns the current barometric pressure (in millibar).
   */
  public float getPressure() {
    return AHRSJNI.c_AHRS_GetPressure();
  }

  /**
   * Returns the current temperature (in degrees centigrade) reported by the sensor's
   * gyro/accelerometer circuit.
   *
   * <p>This value may be useful in order to perform advanced temperature- correction of raw
   * gyroscope and accelerometer values.
   *
   * <p>
   *
   * @return The current temperature (in degrees centigrade).
   */
  public float getTempC() {
    return AHRSJNI.c_AHRS_GetTempC();
  }

  /**
   * Returns information regarding which sensor board axis (X,Y or Z) and direction (up/down) is
   * currently configured to report Yaw (Z) angle values. NOTE: If the board firmware supports
   * Omnimount, the board yaw axis/direction are configurable.
   *
   * <p>For more information on Omnimount, please see:
   *
   * <p>http://navx-mxp.kauailabs.com/navx-mxp/installation/omnimount/
   *
   * <p>
   *
   * @return The currently-configured board yaw axis/direction.
   */
  public BoardYawAxis getBoardYawAxis() {
    return AHRSJNI.c_AHRS_GetBoardYawAxis();
  }

  /**
   * Returns the version number of the firmware currently executing on the sensor.
   *
   * <p>To update the firmware to the latest version, please see:
   *
   * <p>http://navx-mxp.kauailabs.com/navx-mxp/support/updating-firmware/
   *
   * <p>
   *
   * @return The firmware version in the format [MajorVersion].[MinorVersion].[Revision]
   */
  public String getFirmwareVersion() {
    return AHRSJNI.c_AHRS_GetFirmwareVersion();
  }

  /**
   * Returns the navX-Model device's currently configured update rate. Note that the update rate
   * that can actually be realized is a value evenly divisible by the navX-Model device's internal
   * motion processor sample clock (200Hz). Therefore, the rate that is returned may be lower than
   * the requested sample rate.
   *
   * <p>The actual sample rate is rounded down to the nearest integer that is divisible by the
   * number of Digital Motion Processor clock ticks. For instance, a request for 58 Hertz will
   * result in an actual rate of 66Hz (200 / (200 / 58), using integer math.
   *
   * @return Returns the current actual update rate in Hz (cycles per second).
   */
  public int getActualUpdateRate() {
    return AHRSJNI.c_AHRS_GetActualUpdateRate();
  }

  /**
   * Returns the currently requested update rate. rate. Note that not every update rate can actually
   * be realized, since the actual update rate must be a value evenly divisible by the navX-Model
   * device's internal motion processor sample clock (200Hz).
   *
   * <p>To determine the actual update rate, use the {@link #getActualUpdateRate()} method.
   *
   * @return Returns the requested update rate in Hz (cycles per second).
   */
  public int getRequestedUpdateRate() {
    return AHRSJNI.c_AHRS_GetRequestedUpdateRate();
  }

  /**
   * Enables or disables logging (via Console I/O) of AHRS library internal behaviors, including
   * events such as transient communication errors.
   *
   * @param enable true to enable logging; false to disable logging
   */
  public void enableLogging(boolean enable) {
    AHRSJNI.c_AHRS_EnableLogging(enable);
  }

  /**
   * Enables or disables board-level yaw zero (reset) requests. Board-level yaw resets are processed
   * by the sensor board and the resulting yaw angle may not be available to the client software
   * until at least 2 update cycles have occurred. Board-level yaw resets however do maintain
   * synchronization between the yaw angle and the sensor-generated Quaternion and Fused Heading
   * values.
   *
   * <p>Conversely, Software-based yaw resets occur instantaneously; however, Software- based yaw
   * resets do not update the yaw angle component of the sensor-generated Quaternion values or the
   * Fused Heading values.
   *
   * @param enable true to enable board-level yaw resets; false to enable software-based yaw resets.
   */
  public void enableBoardlevelYawReset(boolean enable) {
    AHRSJNI.c_AHRS_EnableBoardlevelYawReset(enable);
  }

  /**
   * Returns true if Board-level yaw resets are enabled. Conversely, returns false if Software-based
   * yaw resets are active.
   *
   * @return true if Board-level yaw resets are enabled; false if software-based yaw resets are
   *     active.
   */
  public boolean isBoardlevelYawResetEnabled() {
    return AHRSJNI.c_AHRS_IsBoardlevelYawResetEnabled();
  }

  /**
   * Returns the sensor full scale range (in degrees per second) of the X, Y and X-axis gyroscopes.
   *
   * @return gyroscope full scale range in degrees/second.
   */
  public short getGyroFullScaleRangeDPS() {
    return AHRSJNI.c_AHRS_GetGyroFullScaleRangeDPS();
  }

  /**
   * Returns the sensor full scale range (in G) of the X, Y and X-axis accelerometers.
   *
   * @return accelerometer full scale range in G.
   */
  public short getAccelFullScaleRangeG() {
    return AHRSJNI.c_AHRS_GetAccelFullScaleRangeG();
  }

  @Override
  public void close() {
    SendableRegistry.remove(this);
    destroy();
  }

  /**
   * Initializes smart dashboard communication
   *
   * @param builder The sendable builder which will be registered with.
   */
  @Override
  public void initSendable(NTSendableBuilder builder) {
    builder.setSmartDashboardType("Gyro");
    builder.addDoubleProperty("Value", this::getYaw, null);
  }
}
