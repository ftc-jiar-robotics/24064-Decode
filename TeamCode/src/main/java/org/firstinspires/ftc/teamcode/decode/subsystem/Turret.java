package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.MAX_VOLTAGE;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.dashTelemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Turret.TurretStates.IDLE;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Turret.TurretStates.ODOM_TRACKING;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Turret.TurretStates.VISION_TRACKING;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.decode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrices.FeedforwardGains;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrices.LowPassGains;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.decode.control.motion.State;
import org.firstinspires.ftc.teamcode.decode.util.AutoAim;
import org.firstinspires.ftc.teamcode.decode.util.LoopUtil;

@Configurable
public class Turret extends Subsystem<Turret.TurretStates> {
    private final MotorEx turret;
    private final Motor.Encoder motorEncoder;
    private final AutoAim autoAim;
    private final VoltageSensor batteryVoltageSensor;

    public static PIDGains odoPIDGains = new PIDGains(
            0.033,
            0.008,
            0.001,
            Double.POSITIVE_INFINITY
    );

    public static PIDGains visionPIDGains = new PIDGains(
            0.023,
            0.007,
            0.0001,
            Double.POSITIVE_INFINITY
    );
    public enum TurretStates {
        IDLE, ODOM_TRACKING, VISION_TRACKING
    }

    private TurretStates currentState = IDLE;

    private final FIRLowPassFilter derivFilter = new FIRLowPassFilter(filterGains);
    private final PIDController odomTracking = new PIDController(derivFilter);
    private final PIDController visionTracking = new PIDController(derivFilter);

    public static FeedforwardGains feedforwardGains = new FeedforwardGains(
            0.005,
            0.002,
            0.0001
    );

    public static LowPassGains filterGains = new LowPassGains(0, 2);

    public static double
            kG = 0,
            TURRET_OFFSET = 2.559,
            TICKS_TO_DEGREES = 90.0 / 148.0,
            WRAP_AROUND_ANGLE = 150,
            PID_TOLERANCE = 1,
            MANUAL_POWER_MULTIPLIER = 0.7,
            TARGET_YAW = 0;

    public static int
            CHECK_UNDETECTED_LOOPS = (1 << 3) - 1, // checking every X loops to switch to VISION_TRACKING state
            CHECK_DETECTED_LOOPS = (1 << 0) - 1; // checking every X loop when in VISION_TRACKING state

    private Pose goal = new Pose(Common.TAG_POSES[0][1], Common.TAG_POSES[0][2]);
    private Pose turretPos = new Pose(0, 0);
    private Pose robotPos = new Pose(0,0);

    private double
            currentAngle = 0.0,
            targetAngle = 0.0,
            robotHeadingTurretDomain = 0,
            autoAimYawOffset = 0.0,
            manualPower = 0.0;

    public Turret(HardwareMap hw) {
        this.turret = new MotorEx(hw, "turret", Motor.GoBILDA.RPM_435);
        this.batteryVoltageSensor = hw.voltageSensor.iterator().next();
        MotorEx rightBack = new MotorEx(hw, "right back", Motor.GoBILDA.RPM_1150);
        autoAim = new AutoAim(hw, Common.isRed, Common.RED_GOAL_ID, Common.BLUE_GOAL_ID, Common.TAG_SIZE_METERS_DECODE, "arduCam");
        motorEncoder = rightBack.encoder;
        motorEncoder.reset();
        odomTracking.setGains(odoPIDGains);
        visionTracking.setGains(visionPIDGains);
        derivFilter.setGains(filterGains);
    }

    public void setGoalAlliance(boolean isRed) {
        goal = isRed ? new Pose(0, 144).mirror() : new Pose(0, 144);

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

    private void setOdomTracking() {
        double theta = calculateAngleToGoal(turretPos);
        double alpha = ((theta - robotHeadingTurretDomain) + 3600) % 360;

        odomTracking.setTarget(new State(targetAngle = normalizeToTurretRange(alpha), 0, 0, 0));
    }

    public boolean isPIDInTolerance() {
        return currentState == ODOM_TRACKING ?
                odomTracking.isPositionInTolerance(new State(currentAngle, 0, 0, 0), PID_TOLERANCE) :
                visionTracking.isPositionInTolerance(new State(autoAim.getTargetYawDegrees(), 0, 0, 0), PID_TOLERANCE);
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
        
        return ((360 - Math.toDegrees(Math.atan2(dy, dx))) + 90 + 3600) % 360;
    }
    // Inputs only from 0 - 360 degrees
    public static double normalizeToTurretRange(double angle) {
        return angle > WRAP_AROUND_ANGLE ? angle - 360 : angle ;
    }

    public static double turretHeadingToStandardHeading(double turretHeadingDeg) {
        return ((90.0 - turretHeadingDeg + 360.0) % 360.0);
    }

    /** Optional reverse conversion:
     *  math radians (0 = East, CCW+) → turret-domain degrees (0 = North, 90° = East, CW+)
     */
    public static double standardHeadingRadToTurretHeading(double standardHeadingDeg) {
        double turretDeg = 90.0 - (standardHeadingDeg);
        turretDeg = (turretDeg % 360.0 + 360.0) % 360.0;
        return turretDeg;
    }

    public void setManual(double power) {
        manualPower = power * MANUAL_POWER_MULTIPLIER;
    }

    @Override
    public void run() {
        currentAngle = motorEncoder.getPosition() * TICKS_TO_DEGREES;

        double scalar = MAX_VOLTAGE / batteryVoltageSensor.getVoltage();
        double output = Math.abs(currentAngle - targetAngle) >= 2 ? kG * scalar : 0;

        odomTracking.setGains(odoPIDGains);
        visionTracking.setGains(visionPIDGains);
        derivFilter.setGains(filterGains);

        // turning robot heading to turret heading
        double robotHeading = Common.robot.drivetrain.getHeading();
        robotHeadingTurretDomain = ((360 - Math.toDegrees(robotHeading)) + 90 + 3600) % 360;
        turretPos = calculateTurretPosition(Common.robot.drivetrain.getPose(), ((360 - Math.toDegrees(robotHeadingTurretDomain)) + 90 + 360) % 360, TURRET_OFFSET);

        if (Math.abs(manualPower) > 0) turret.set(manualPower);

        else {
            switch (currentState) {
                case IDLE:
                    targetAngle = 0;
                    break;
                case ODOM_TRACKING:
                    setOdomTracking();
                    output += odomTracking.calculate(new State(currentAngle, 0, 0 ,0));
                    if ((LoopUtil.getLoops() & CHECK_UNDETECTED_LOOPS) == 0) {
                        if (autoAim.isTargetDetected()) currentState = TurretStates.VISION_TRACKING;
                        else break;
                    } else {
                        break;
                    }
                    break;
                case VISION_TRACKING:
                    if ((LoopUtil.getLoops() & CHECK_DETECTED_LOOPS) == 0) {
                        if (autoAim.isTargetDetected()) {
                            visionTracking.setTarget(new State(TARGET_YAW,0,0,0));
                            robotPos = autoAim.getRobotPos(Math.toRadians(currentAngle));
                            autoAimYawOffset = autoAim.getYawOffsetRadians(Math.toRadians(turretHeadingToStandardHeading(currentAngle)));
                            output += visionTracking.calculate(new State(autoAimYawOffset, 0, 0, 0));
                        } else currentState = ODOM_TRACKING;
                    }
                    if (currentAngle < WRAP_AROUND_ANGLE - 360 || currentAngle > WRAP_AROUND_ANGLE) currentState = ODOM_TRACKING;
                    break;
            }


            rawPower = output;

            boolean odomTrackingInTolerance = currentState == ODOM_TRACKING && odomTracking.isPositionInTolerance(new State(currentAngle, 0, 0, 0), 0.2);
            boolean visionTrackingInTolerance =currentState == VISION_TRACKING && visionTracking.isPositionInTolerance(new State(currentAngle, 0, 0, 0), 0.2);

            if (odomTrackingInTolerance || visionTrackingInTolerance) turret.set(0);
            else turret.set(output);

        }


        if (isPIDInTolerance()) odomTracking.reset();
    }


    double rawPower = 0;
    public void printTelemetry() {
        telemetry.addLine("TURRET");
        telemetry.addData("current state: ", currentState);
        telemetry.addData("turret domain robot heading: ", ((360 - Math.toDegrees(Common.robot.drivetrain.getHeading())) + 90 + 360) % 360);
        telemetry.addData("calculated theta: ", calculateAngleToGoal(turretPos));
        telemetry.addData("turret pos x: ", turretPos.getX());
        telemetry.addData("turret pos y: ", turretPos.getY());
        telemetry.addData("raw ticks", motorEncoder.getPosition());
        telemetry.addData("raw power", rawPower);
        telemetry.addData("calculated distance: ", getDistance());
        telemetry.addData("is PID in tolerance: ", isPIDInTolerance());

        dashTelemetry.addLine("TURRET");
        dashTelemetry.addData("vision setpoint: ", 0);
        dashTelemetry.addData("current vision: ", autoAim.getTargetYawDegrees());
        dashTelemetry.addData("encoder angle: ", currentAngle);
        dashTelemetry.addData("target angle: ", targetAngle);
        dashTelemetry.addData("yaw offset (degrees): ", Math.toDegrees(autoAimYawOffset));
        dashTelemetry.addData("pose from camera", robotPos);

    }
}
