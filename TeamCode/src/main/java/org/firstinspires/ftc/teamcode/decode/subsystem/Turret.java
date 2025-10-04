package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.MAX_VOLTAGE;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.graph;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import android.sax.StartElementListener;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.PanelsGraph;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.decode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrices.FeedforwardGains;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrices.LowPassGains;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.decode.control.motion.State;

@Configurable
public class Turret extends Subsystem<Double> {
    private final MotorEx turret;
    private final AnalogInput encoder;
    private final Motor.Encoder motorEncoder;


    private final VoltageSensor batteryVoltageSensor;

    public static PIDGains pidGains = new PIDGains(
            0.008,
            0.005,
            0.0001,
            Double.POSITIVE_INFINITY
    );
    private final FIRLowPassFilter derivFilter = new FIRLowPassFilter(filterGains);
    private final PIDController controller = new PIDController(derivFilter);

    public static FeedforwardGains feedforwardGains = new FeedforwardGains(
            0.005,
            0.002,
            0.0001
    );

    public static LowPassGains filterGains = new LowPassGains(0, 2);

    public static double
            offset = -90,
            kG = 0,
            turretOffset = 2,
            ticksToDegrees = 90.0/148.0,
            warpAroundAngle = 159;

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
        controller.setGains(pidGains);
        derivFilter.setGains(filterGains);

    }

    @Override
    public void set(Double a) {
        targetAngle = normalizeFrom0to360((AngleUnit.normalizeDegrees(a) + 360) % 360);
        controller.setTarget(new State(targetAngle, 0, 0, 0));
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

        controller.setTarget(null); //TODO fix me!
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

    public static double normalizeFrom0to360(double angle) {
        return angle > warpAroundAngle ? angle - 360 : angle ;
    }

    @Override
    public void run() {
        currentAngle = ((motorEncoder.getPosition() * ticksToDegrees) + 360) % 360;//AngleUnit.normalizeDegrees((encoder.getVoltage()-0.043)/3.1*360 + offset);
        currentAngle = normalizeFrom0to360(currentAngle);


        double scalar = MAX_VOLTAGE / batteryVoltageSensor.getVoltage();
        double output = Math.abs(currentAngle - targetAngle) >= 2 ? kG * scalar : 0;

        if (manualPower != 0) {
            output = manualPower;
        } else {
            controller.setGains(pidGains);
            derivFilter.setGains(filterGains);
            output += controller.calculate(new State(currentAngle, 0, 0 ,0)); // derivFilter.calculate
        }

        rawPower = output;
        turret.set(output);
    }

    double rawPower = 0;
    public void printTelemetry() {
        telemetry.addData("encoder angle: ", currentAngle);
        telemetry.addData("target angle: ", targetAngle);
        telemetry.addData("raw ticks", motorEncoder.getPosition());
        telemetry.addData("raw power", rawPower);
        telemetry.addData("calculated distance: ", getDistance());
        graph.addData("target angle", targetAngle);
        graph.addData("encoder angle", currentAngle);
    }
}
