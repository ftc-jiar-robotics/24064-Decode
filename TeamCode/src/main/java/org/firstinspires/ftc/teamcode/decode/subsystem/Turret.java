package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.MAX_VOLTAGE;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.graph;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.decode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrices.FeedforwardGains;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrices.LowPassGains;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.decode.control.motion.State;

@Configurable
public class Turret extends Subsystem<Turret.TurretStates> {
    private final MotorEx turret;
    private final AnalogInput encoder;
    private final Motor.Encoder motorEncoder;


    private final VoltageSensor batteryVoltageSensor;

    public static PIDGains pidGains = new PIDGains(
            0.025,
            0.0030,
            0.00065,
            Double.POSITIVE_INFINITY
    );

    public enum TurretStates {
        IDLE, ODOM_TRACKING, VISION_TRACKING
    }

    private TurretStates currentState = TurretStates.IDLE;

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
            turretOffset = 2.559,
            ticksToDegrees = 90.0/148.0,
            wrapAroundAngle = 159,
            pidTolerance = 2; // TODO tune in angle measurement

    public static Pose
            goal = new Pose(13.4, 136.8);

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
    public void set(TurretStates a) {
        currentState = a;
    }

    @Override
    public TurretStates get() {
        return currentState;
    }

    public double getDistance() {
        double dx = goal.getX() - turretPos.getX();
        double dy = goal.getY() - turretPos.getY();
        return Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
    }

    private void setTracking() {
        // turning robot heading to turret heading
        double robotHeading = Common.robot.drivetrain.getHeading();
        double robotHeadingTurretDomain = ((360 - Math.toDegrees(robotHeading)) + 90 + 360) % 360;

        turretPos = calculateTurretPosition(Common.robot.drivetrain.getPose(), ((360 - Math.toDegrees(robotHeadingTurretDomain)) + 90 + 360) % 360, turretOffset);

        double theta = calculateAngleToGoal(turretPos);
        double alpha = theta - robotHeadingTurretDomain;

        controller.setTarget(new State(targetAngle = normalizeFrom0to360(alpha), 0, 0, 0)); //TODO fix me!
    }

    public boolean isPIDInTolerance() {
        return controller.isPositionInTolerance(new State(currentAngle, 0, 0, 0), pidTolerance);
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
    public double calculateAngleToGoal(Pose turretPos) {
        double dx = goal.getX() - turretPos.getX();
        double dy = goal.getY() - turretPos.getY();
        return ((360-Math.toDegrees(Math.atan2(dy, dx)))+90+360)%360;
    }

    public static double normalizeFrom0to360(double angle) {
        return angle > wrapAroundAngle ? angle - 360 : angle ;
    }

    @Override
    public void run() {
        currentAngle = motorEncoder.getPosition() * ticksToDegrees;

        double scalar = MAX_VOLTAGE / batteryVoltageSensor.getVoltage();
        double output = Math.abs(currentAngle - targetAngle) >= 2 ? kG * scalar : 0;

        if (manualPower != 0) {
            output = manualPower;
        } else {
            controller.setGains(pidGains);
            derivFilter.setGains(filterGains);

            output += controller.calculate(new State(currentAngle, 0, 0 ,0));
        }

        switch (currentState) {
            case IDLE:
                targetAngle = 0;
                break;
            case ODOM_TRACKING:
                setTracking();
                break;
            case VISION_TRACKING:
                break;
        }

        rawPower = output;
        turret.set(output);
    }

    double rawPower = 0;
    public void printTelemetry() {
        telemetry.addLine("TURRET");
        telemetry.addData("encoder angle: ", currentAngle);
        telemetry.addData("target angle: ", targetAngle);
        telemetry.addData("turret domain robot heading: ", ((360 - Math.toDegrees(Common.robot.drivetrain.getHeading())) + 90 + 360) % 360);
        telemetry.addData("calculated theta: ", calculateAngleToGoal(turretPos));
        telemetry.addData("turret pos x: ", turretPos.getX());
        telemetry.addData("turret pos y: ", turretPos.getY());
        telemetry.addData("raw ticks", motorEncoder.getPosition());
        telemetry.addData("raw power", rawPower);
        telemetry.addData("calculated distance: ", getDistance());
        graph.addData("target angle", targetAngle);
        graph.addData("encoder angle", currentAngle);
        telemetry.addData("is PID in tolerance: ", isPIDInTolerance());
    }
}
