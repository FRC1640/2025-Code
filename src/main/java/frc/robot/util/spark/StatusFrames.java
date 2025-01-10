package frc.robot.util.spark;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SignalsConfig;

public class StatusFrames {
  int faults,
      absEnPos,
      absEnVel,
      analPos,
      analVel,
      analVolt,
      motorTemp,
      applOutput,
      busVolt,
      altEnPos,
      altEnVel,
      iAccum,
      limits,
      outCurr,
      primEnPos,
      primEnVel,
      warnings;

  public StatusFrames(
      int status0, int status1, int status2, int status3, int status4, int status5, int status6) {
    this.faults = status0;
    this.absEnPos = status5;
    this.absEnVel = status6;
    this.analPos = status3;
    this.analVel = status3;
    this.analVolt = status3;
    this.motorTemp = status1;
    this.applOutput = status0;
    this.altEnPos = status4;
    this.altEnVel = status4;
    this.outCurr = status1;
    this.busVolt = 10;
    this.iAccum = 20;
    this.limits = 10;
    this.primEnPos = status2;
    this.primEnVel = status2;
    this.warnings = 250;
  }

  /**
   * Sets periodic frame rates for the given parameters. Using status frame framework, status0
   * corresponds to faults and applied output, and status1 to motor temperature and output current.
   * The remaining 5 statuses configure encoder parameters. Configuration of these parameters
   * separately between different types of encoders is not supported by this method but should not
   * be necessary as only one encoder's parameters will be relevant. Set these parameters through
   * {@code encoderPos}, {@code encoderVel}, and {@code encoderVolt}.
   *
   * @param faults Spark fault framerate.
   * @param encoderPos Encoder position. Configures frame rates for absolute, alternate, analog, and
   *     primary encoders to the same value simultaneously.
   * @param encoderVel Encoder velocity. Frame rates will be configured for absolute, alternate,
   *     analog, and primary encoders to the same value simultaneously.
   * @param encoderVolt Analog encoder voltage reading.
   * @param motorTemp Motor temperature frame rate.
   * @param applOutput Applied output frame rate.
   * @param busVolt Bus voltage frame rate.
   * @param iAccum PID controller integral accumulation frame frate.
   * @param limits Limit switch frame rate.
   * @param outCurr Output current frame rate.
   * @param warnings Warnings frame rate.
   */
  public StatusFrames(
      int faults,
      int encoderPos,
      int encoderVel,
      int encoderVolt,
      int motorTemp,
      int applOutput,
      int busVolt,
      int iAccum,
      int limits,
      int outCurr,
      int warnings) {
    this.faults = faults;
    this.absEnPos = encoderPos;
    this.absEnVel = encoderVel;
    this.analPos = encoderPos;
    this.analVel = encoderVel;
    this.analVolt = encoderVolt;
    this.motorTemp = motorTemp;
    this.applOutput = applOutput;
    this.busVolt = busVolt;
    this.altEnPos = encoderPos;
    this.altEnVel = encoderVel;
    this.iAccum = iAccum;
    this.limits = limits;
    this.outCurr = outCurr;
    this.primEnPos = encoderPos;
    this.primEnVel = encoderVel;
    this.warnings = warnings;
  }

  public void setNonStatusFrameParameters(int busVolt, int iAccum, int limits, int warnings) {
    this.busVolt = busVolt;
    this.iAccum = iAccum;
    this.limits = limits;
    this.warnings = warnings;
  }

  public void apply(SignalsConfig signals) {
    signals
        .faultsPeriodMs(faults)
        .absoluteEncoderPositionPeriodMs(absEnPos)
        .absoluteEncoderVelocityPeriodMs(absEnVel)
        .analogPositionPeriodMs(analPos)
        .analogVelocityPeriodMs(analVel)
        .analogVoltagePeriodMs(analVolt)
        .motorTemperaturePeriodMs(motorTemp)
        .appliedOutputPeriodMs(applOutput)
        .busVoltagePeriodMs(busVolt)
        .externalOrAltEncoderPosition(altEnPos)
        .externalOrAltEncoderVelocity(altEnVel)
        .iAccumulationPeriodMs(iAccum)
        .limitsPeriodMs(limits)
        .outputCurrentPeriodMs(outCurr)
        .primaryEncoderPositionPeriodMs(primEnPos)
        .primaryEncoderVelocityPeriodMs(primEnVel)
        .warningsPeriodMs(warnings);
  }

  public boolean getFlashNecessary(SparkMax spark) {
    return ((spark.configAccessor.signals.getFaultsPeriodMs() != faults)
        || (spark.configAccessor.signals.getAbsoluteEncoderPositionPeriodMs() != absEnPos)
        || (spark.configAccessor.signals.getAbsoluteEncoderVelocityPeriodMs() != absEnVel)
        || (spark.configAccessor.signals.getAnalogPositionPeriodMs() != analPos)
        || (spark.configAccessor.signals.getAnalogVelocityPeriodMs() != analVel)
        || (spark.configAccessor.signals.getAnalogVelocityPeriodMs() != analVolt)
        || (spark.configAccessor.signals.getMotorTemperaturePeriodMs() != motorTemp)
        || (spark.configAccessor.signals.getAppliedOutputPeriodMs() != applOutput)
        || (spark.configAccessor.signals.getBusVoltagePeriodMs() != busVolt)
        || (spark.configAccessor.signals.getExternalOrAltEncoderPositionPeriodMs() != altEnPos)
        || (spark.configAccessor.signals.getExternalOrAltEncoderVelocityPeriodMs() != altEnVel)
        || (spark.configAccessor.signals.getIAccumulationPeriodMs() != iAccum)
        || (spark.configAccessor.signals.getLimitsPeriodMs() != limits)
        || (spark.configAccessor.signals.getOutputCurrentPeriodMs() != outCurr)
        || (spark.configAccessor.signals.getPrimaryEncoderPositionPeriodMs() != primEnPos)
        || (spark.configAccessor.signals.getPrimaryEncoderVelocityPeriodMs() != primEnVel)
        || (spark.configAccessor.signals.getWarningsPeriodMs() != warnings));
  }
}
