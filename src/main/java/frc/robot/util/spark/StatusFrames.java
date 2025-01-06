package frc.robot.util.spark;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SignalsConfig;

public class StatusFrames {
    int faults, absEnPos, absEnVel,
        analPos, analVel, analVolt,
        motorTemp, applOutput, busVolt,
        altEnPos, altEnVel, iAccum,
        limits, outCurr, primEnPos,
        primEnVel, warnings;

    public StatusFrames(int status0, int status1, int status2,
            int status3, int status4, int status5, int status6) {
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
        this.primEnPos = 20;
        this.primEnVel = 20;
        this.warnings = 250;
    }

    public StatusFrames(int status0, int status1, int status2,
            int status3, int status4, int status5, int status6,
            int busVolt, int iAccum, int limits, int primEnPos,
            int primEnVel, int warnings) {
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
        this.busVolt = busVolt;
        this.iAccum = iAccum;
        this.limits = limits;
        this.primEnPos = primEnPos;
        this.primEnVel = primEnVel;
        this.warnings = warnings;
    }

    public StatusFrames(int faults, int absEnPos, int absEnVel,
            int analPos, int analVal, int analVolt, int motorTemp,
            int applOutput, int busVolt, int altEnPos, int altEnVel,
            int iAccum, int limits, int outCurr, int primEnPos,
            int primEnVel, int warnings) {
        this.faults = faults;
        this.absEnPos = absEnPos;
        this.absEnVel = absEnVel;
        this.analPos = analPos;
        this.analVel = analVal;
        this.analVolt = analVolt;
        this.motorTemp = motorTemp;
        this.applOutput = applOutput;
        this.busVolt = busVolt;
        this.altEnPos = altEnPos;
        this.altEnVel = altEnVel;
        this.iAccum = iAccum;
        this.limits = limits;
        this.outCurr = outCurr;
        this.primEnPos = primEnPos;
        this.primEnVel = primEnVel;
        this.warnings = warnings;
    }

    public void apply(SignalsConfig signals) {
        signals.faultsPeriodMs(faults);
        signals.absoluteEncoderPositionPeriodMs(absEnPos);
        signals.absoluteEncoderVelocityPeriodMs(absEnVel);
        signals.analogPositionPeriodMs(analPos);
        signals.analogVelocityPeriodMs(analVel);
        signals.analogVoltagePeriodMs(analVolt);
        signals.motorTemperaturePeriodMs(motorTemp);
        signals.appliedOutputPeriodMs(applOutput);
        signals.busVoltagePeriodMs(busVolt);
        signals.externalOrAltEncoderPosition(altEnPos);
        signals.externalOrAltEncoderVelocity(altEnVel);
        signals.iAccumulationPeriodMs(iAccum);
        signals.limitsPeriodMs(limits);
        signals.outputCurrentPeriodMs(outCurr);
        signals.primaryEncoderPositionPeriodMs(primEnPos);
        signals.primaryEncoderVelocityPeriodMs(primEnVel);
        signals.warningsPeriodMs(warnings);
    }

    public boolean getFlashNecessary(SparkMax spark) {
        return (
            (spark.configAccessor.signals.getFaultsPeriodMs() != faults) ||
            (spark.configAccessor.signals.getAbsoluteEncoderPositionPeriodMs() != absEnPos) ||
            (spark.configAccessor.signals.getAbsoluteEncoderVelocityPeriodMs() != absEnVel) ||
            (spark.configAccessor.signals.getAnalogPositionPeriodMs() != analPos) ||
            (spark.configAccessor.signals.getAnalogVelocityPeriodMs() != analVel) ||
            (spark.configAccessor.signals.getAnalogVelocityPeriodMs() != analVolt) ||
            (spark.configAccessor.signals.getMotorTemperaturePeriodMs() != motorTemp) ||
            (spark.configAccessor.signals.getAppliedOutputPeriodMs() != applOutput) ||
            (spark.configAccessor.signals.getBusVoltagePeriodMs() != busVolt) ||
            (spark.configAccessor.signals.getExternalOrAltEncoderPositionPeriodMs() != altEnPos) ||
            (spark.configAccessor.signals.getExternalOrAltEncoderVelocityPeriodMs() != altEnVel) ||
            (spark.configAccessor.signals.getIAccumulationPeriodMs() != iAccum) ||
            (spark.configAccessor.signals.getLimitsPeriodMs() != limits) ||
            (spark.configAccessor.signals.getOutputCurrentPeriodMs() != outCurr) ||
            (spark.configAccessor.signals.getPrimaryEncoderPositionPeriodMs() != primEnPos) ||
            (spark.configAccessor.signals.getPrimaryEncoderVelocityPeriodMs() != primEnVel) ||
            (spark.configAccessor.signals.getWarningsPeriodMs() != warnings)
        );
    }

    public boolean getFlashNecessary(SparkFlex spark) {
        return (
            (spark.configAccessor.signals.getFaultsPeriodMs() != faults) ||
            (spark.configAccessor.signals.getAbsoluteEncoderPositionPeriodMs() != absEnPos) ||
            (spark.configAccessor.signals.getAbsoluteEncoderVelocityPeriodMs() != absEnVel) ||
            (spark.configAccessor.signals.getAnalogPositionPeriodMs() != analPos) ||
            (spark.configAccessor.signals.getAnalogVelocityPeriodMs() != analVel) ||
            (spark.configAccessor.signals.getAnalogVelocityPeriodMs() != analVolt) ||
            (spark.configAccessor.signals.getMotorTemperaturePeriodMs() != motorTemp) ||
            (spark.configAccessor.signals.getAppliedOutputPeriodMs() != applOutput) ||
            (spark.configAccessor.signals.getBusVoltagePeriodMs() != busVolt) ||
            (spark.configAccessor.signals.getExternalOrAltEncoderPositionPeriodMs() != altEnPos) ||
            (spark.configAccessor.signals.getExternalOrAltEncoderVelocityPeriodMs() != altEnVel) ||
            (spark.configAccessor.signals.getIAccumulationPeriodMs() != iAccum) ||
            (spark.configAccessor.signals.getLimitsPeriodMs() != limits) ||
            (spark.configAccessor.signals.getOutputCurrentPeriodMs() != outCurr) ||
            (spark.configAccessor.signals.getPrimaryEncoderPositionPeriodMs() != primEnPos) ||
            (spark.configAccessor.signals.getPrimaryEncoderVelocityPeriodMs() != primEnVel) ||
            (spark.configAccessor.signals.getWarningsPeriodMs() != warnings)
        );
    }
}
