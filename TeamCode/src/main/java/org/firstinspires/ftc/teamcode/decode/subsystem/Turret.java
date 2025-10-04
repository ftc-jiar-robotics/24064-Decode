package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.MAX_VOLTAGE;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import static java.lang.Math.PI;

import android.util.Range;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrices.FeedforwardGains;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrices.LowPassGains;

import java.util.stream.Stream;

@Configurable
public class Turret extends Subsystem<Double> {
    private final MotorEx turret;
    private final AnalogInput encoder;
    private final Motor.Encoder motorEncoder;


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
            offset = -90,
            kG = 0,
            kP = 0,
            kI = 0,
            kD = 0,
            kF = 0,
            turretOffset = 2,
            ticksToDegrees = 90.0/148.0;

    public static Pose
            goal = new Pose(10, 10),
            robot = new Pose(5, 5);

    private Pose turretPos = new Pose(0, 0);

    private double
            currentAngle = 0.0,
            targetAngle = 0.0,
            manualPower = 0.0;

    public Turret(HardwareMap hw) {
        this.turret = new MotorEx(hw, "turret", Motor.GoBILDA.RPM_435);
        this.encoder = hw.get(AnalogInput.class, "turretEncoder");
        this.batteryVoltageSensor = hw.voltageSensor.iterator().next();
        MotorEx rightBack = new MotorEx(hw, "right back", Motor.GoBILDA.RPM_1150);

        motorEncoder = rightBack.encoder;
        motorEncoder.reset();
        derivFilter.setGains(filterGains);
    }

    @Override
    public void set(Double a) {
        targetAngle = AngleUnit.normalizeDegrees(a);
        pidfController.setSetPoint(targetAngle);
    }

    @Override
    public Double get() {
        return currentAngle;
    }

    public double getDistance() {
        return Math.sqrt(Math.pow(turretPos.getX(), 2) + Math.pow(turretPos.getY(), 2));
    }

    public void setManualPower(Double p) {
        manualPower = p;
    }

    public boolean setTracking(double robotHeading) {
        turretPos = calculateTurretPosition(robot, robotHeading, turretOffset);

        double theta = calculateAngleToGoal(turretPos, goal);
        double alpha = theta - robotHeading;

        pidfController.setSetPoint(AngleUnit.normalizeDegrees(alpha));
        return true;
    }

    /**
     * Calculate the turret position (xt, yt).
     * Formula: xt = x - cos(heading) * D, yt = y - sin(heading) * D
     */
    public Pose calculateTurretPosition(Pose robotPos, double headingDeg, double offset) {
        double headingRad = Math.toRadians(headingDeg);

        double xOffset = Math.cos(headingRad) * offset;
        double yOffset = Math.sin(headingRad) * offset;

        double xt = robotPos.getX() - xOffset;
        double yt = robotPos.getY() - yOffset;

        return new Pose(xt, yt);
    }

    /**
     * Calculate θ, the raw angle from turret to goal.
     * Formula: atan2(yg - yt, xg - xt)
     */
    public double calculateAngleToGoal(Pose turretPos, Pose goalPos) {
        double dx = goalPos.getX() - turretPos.getX();
        double dy = goalPos.getY() - turretPos.getY();
        return Math.toDegrees(Math.atan2(dy, dx));
    }

    public static double map(double value, double in_min, double in_max, double out_min, double out_max) {
        return out_min + ((value - in_min) / (in_max - in_min)) * (out_max - out_min);
    }

    @Override
    public void run() {
        currentAngle = ((motorEncoder.getPosition()*ticksToDegrees)+360)%360;//AngleUnit.normalizeDegrees((encoder.getVoltage()-0.043)/3.1*360 + offset);
        currentAngle = map(currentAngle, 0, 360, 20, 380);


        double scalar = MAX_VOLTAGE / batteryVoltageSensor.getVoltage();
        double output = Math.abs(currentAngle - targetAngle) >= 2 ? kG * scalar : 0;

        if (manualPower != 0) {
            output += manualPower;
        } else {
            output += derivFilter.calculate(pidfController.calculate(currentAngle));
        }

        turret.set(output);
    }

    public void printTelemetry() {
        telemetry.addData("encoder angle: ", currentAngle);
        telemetry.addData("target angle: ", targetAngle);
        telemetry.addData("raw ticks", motorEncoder.getPosition());
        telemetry.addData("calculated distance: ", getDistance());
    }
}
