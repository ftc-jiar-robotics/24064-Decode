package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.MAX_VOLTAGE;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrices.FeedforwardGains;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrices.LowPassGains;

public class Turret extends Subsystem<Double> {
    private final MotorEx turret;
    private final AnalogInput encoder;

    private final VoltageSensor batteryVoltageSensor;

    private final PIDFController pidfController = new PIDFController(kP, kI, kD, kF);
    private final FIRLowPassFilter derivFilter = new FIRLowPassFilter(filterGains);

    public static FeedforwardGains feedforwardGains = new FeedforwardGains(
            0.005,
            0.002,
            0.0001
    );

    public static LowPassGains filterGains = new LowPassGains(0, 2);

    public static double
            offset = 0,
            kG = 0,
            kP = 0,
            kI = 0,
            kD = 0,
            kF = 0;

    private double
            currentAngle = 0.0,
            targetAngle = 0.0,
            manualPower = 0.0;

    public Turret(HardwareMap hw) {
        this.turret = new MotorEx(hw, "turret", Motor.GoBILDA.RPM_435);
        this.encoder = hw.get(AnalogInput.class, "turretEncoder");
        this.batteryVoltageSensor = hw.voltageSensor.iterator().next();

        derivFilter.setGains(filterGains);
    }

    public void set(Double a) {
        targetAngle = AngleUnit.normalizeDegrees(a);
        pidfController.setSetPoint(targetAngle);
    }

    public Double get() {
        return currentAngle;
    }

    public void setManualPower(Double p) {
        manualPower = p;
    }

    public void run() {
        currentAngle = AngleUnit.normalizeDegrees((encoder.getVoltage()-0.043)/3.1*360 + offset);

        double scalar = MAX_VOLTAGE / batteryVoltageSensor.getVoltage();
        double output = Math.abs(currentAngle - targetAngle) >= 2 ? kG * scalar : 0;

        if (manualPower != 0) {
            output += manualPower;
        } else {
            output += derivFilter.calculate(pidfController.calculate(currentAngle));
        }

        turret.set(output);
    }
}
