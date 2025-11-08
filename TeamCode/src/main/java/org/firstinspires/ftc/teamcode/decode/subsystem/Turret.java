package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.MAX_VOLTAGE;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_TURRET_CAMERA;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_TURRET_ENCODER;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_TURRET_MOTOR;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.dashTelemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Turret.TurretStates.IDLE;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Turret.TurretStates.VISION_TRACKING;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.decode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.decode.control.filter.singlefilter.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrices.FeedforwardGains;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrices.LowPassGains;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.decode.control.motion.State;
import org.firstinspires.ftc.teamcode.decode.util.AutoAim;
import org.firstinspires.ftc.teamcode.decode.util.CachedMotor;
import org.firstinspires.ftc.teamcode.decode.util.LoopUtil;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;

@Configurable
public class Turret extends Subsystem<Turret.TurretStates> {
    private final CachedMotor turret;
    private final AnalogInput absoluteEncoder;
    private final Motor.Encoder motorEncoder;
    private final AutoAim autoAim;
    public static PIDGains odoPIDGains = new PIDGains(
            0.033,
            0.008,
            0.001,
            Double.POSITIVE_INFINITY
    );

    public enum TurretStates {
        IDLE, ODOM_TRACKING, VISION_TRACKING
    }

    private TurretStates currentState = IDLE;

    private final FIRLowPassFilter derivFilter = new FIRLowPassFilter(filterGains);
    private final PIDController controller = new PIDController(derivFilter);

    public static LowPassGains filterGains = new LowPassGains(0, 2);

    private Queue<Pose> visionSamplePoses = new LinkedList<>();
    private double[] visionVariances = new double[3];

    public static double
            kG = 0,
            TICKS_TO_DEGREES = 90.0 / 148.0,
            WRAP_AROUND_ANGLE = 150,
            VARIANCE_TOLERANCE = 0.04,
            HEADING_VARIANCE_TOLERANCE = 0.04,
            VISION_SAMPLE_SIZE = 5,
            PID_TOLERANCE = 2,
            MANUAL_POWER_MULTIPLIER = 0.7,
            ABSOLUTE_ENCODER_OFFSET = -33.0;

    public static int
            CHECK_UNDETECTED_LOOPS = (1 << 3) - 1, // checking every X loops to switch to VISION_TRACKING state
            CHECK_DETECTED_LOOPS = (1 << 0) - 1; // checking every X loop when in VISION_TRACKING state

    private Pose goal = new Pose(0, 144);
    private Pose turretPos = new Pose(0, 0);
    private Pose robotPoseFromVision = new Pose(0, 0);

    private double
            currentAngle = 0.0,
            targetAngle = 0.0,
            encoderOffset = 0.0,
            robotHeadingTurretDomain = 0.0,
            rawPower = 0.0,
            manualPower = 0.0;

    public Turret(HardwareMap hw) {
        this.turret = new CachedMotor(hw, NAME_TURRET_MOTOR, Motor.GoBILDA.RPM_1150);
        MotorEx rightBack = new MotorEx(hw, "right back", Motor.GoBILDA.RPM_1150);
        absoluteEncoder = hw.get(AnalogInput.class, NAME_TURRET_ENCODER);

        autoAim = new AutoAim(hw, NAME_TURRET_CAMERA);
        motorEncoder = rightBack.encoder;

        motorEncoder.reset();
        controller.setGains(odoPIDGains);
        derivFilter.setGains(filterGains);
    }

    public void setGoalAlliance() {
        goal = Common.isRed ? new Pose(0, 141).mirror() : new Pose(0, 141);
        autoAim.setAlliance();
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
        return Math.sqrt(dx * dx + dy * dy);
    }


    public void applyOffset() {
        encoderOffset = motorEncoder.getPosition() * TICKS_TO_DEGREES - normalizeToTurretRange((360 - ((absoluteEncoder.getVoltage() / 3.2 * 360 + ABSOLUTE_ENCODER_OFFSET) % 360)) % 360);
    }

    private void setTracking() {
        double theta = calculateAngleToGoal(turretPos);
        double alpha = ((theta - robotHeadingTurretDomain) + 3600) % 360;

        controller.setTarget(new State(targetAngle = normalizeToTurretRange(alpha), 0, 0, 0));
    }

    public boolean isPIDInTolerance() {
        return controller.isPositionInTolerance(new State(currentAngle, 0, 0, 0), PID_TOLERANCE);
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
        return angle > WRAP_AROUND_ANGLE ? angle - 360 : angle;
    }

    public void setManual(double power) {
        manualPower = power * MANUAL_POWER_MULTIPLIER;
    }

    @Override
    public void run() {
        currentAngle = (motorEncoder.getPosition() * TICKS_TO_DEGREES) - encoderOffset;

        double scalar = MAX_VOLTAGE / robot.batteryVoltageSensor.getVoltage();
        double output = Math.abs(currentAngle - targetAngle) >= 2 ? kG * scalar : 0;

        controller.setGains(odoPIDGains);
        derivFilter.setGains(filterGains);
        // turning robot heading to turret heading
        double robotHeading = robot.drivetrain.getHeading();
        robotHeadingTurretDomain = ((360 - Math.toDegrees(robotHeading)) + 90 + 3600) % 360;

        if (Math.abs(manualPower) > 0) turret.set(manualPower);

        else {
            switch (currentState) {
                case IDLE:
                    targetAngle = 0;
                    output += controller.calculate(new State(currentAngle, 0, 0 ,0));
                    break;
                case ODOM_TRACKING:
                    turretPos = calculateTurretPosition(robot.drivetrain.getPose(), Math.toDegrees(robotHeading), -Common.TURRET_OFFSET_Y);
                    setTracking();
                    output = controller.calculate(new State(currentAngle, 0, 0 ,0));
                    if ((LoopUtil.getLoops() & CHECK_UNDETECTED_LOOPS) == 0) {
                        if (autoAim.isTargetDetected()) currentState = TurretStates.VISION_TRACKING;
                        else break;
                    } else {
                        break;
                    }
                    break;
                case VISION_TRACKING:
                    if ((LoopUtil.getLoops() & CHECK_DETECTED_LOOPS) == 0) {
                        if (!autoAim.isTargetDetected()) {
                            currentState = TurretStates.ODOM_TRACKING;
                            break;
                        }

                        turretPos = autoAim.getTurretPosePedro();
                        robotPoseFromVision = relocalizeRobotFromTurret(turretPos, robot.drivetrain.getHeading());

                        visionSamplePoses.add(robotPoseFromVision);
                        if (visionSamplePoses.size() >= VISION_SAMPLE_SIZE) visionSamplePoses.remove();

                        visionVariances = getVariance(visionSamplePoses);

//                        if (visionVariances[0] < VARIANCE_TOLERANCE && visionVariances[1] < VARIANCE_TOLERANCE && visionVariances[2] < Math.toRadians(HEADING_VARIANCE_TOLERANCE))
//                            robot.drivetrain.setPose(robotPoseFromVision);

                        setTracking();
                        output = controller.calculate(new State(currentAngle, 0, 0, 0));

                        break;
                    }
            }


            rawPower = output;

            boolean pidInTolerance = controller.isPositionInTolerance(
                    new State(currentAngle, 0, 0, 0), 0.2
            );

            turret.set(pidInTolerance ? 0 : output);

        }

        if (isPIDInTolerance()) controller.reset();
    }

    // Pedro frame: 0° = North (+Y), 90° = East (+X), CCW+
    public static Pose relocalizeRobotFromTurret(Pose turretPosPedro, double robotHeading) {
        double d = Common.TURRET_OFFSET_Y;
        double x = turretPosPedro.getX() - d * Math.cos(robotHeading);
        double y = turretPosPedro.getY() - d * Math.sin(robotHeading);
        return new Pose(x, y, robotHeading);
    }

    public static double[] getVariance(Queue<Pose> queue) {
        double meanY = 0;
        double meanX = 0;
        double meanHeading = 0;

        double xVariance = 0;
        double yVariance = 0;
        double headingVariance = 0;

        double size = queue.size();

        for (Pose p : queue) {
            meanY += p.getY();
            meanX += p.getX();
            meanHeading += p.getHeading();
        }

        meanY /= size;
        meanX /= size;
        meanHeading /= size;

        for (Pose p : queue) {
            double pX = p.getX();
            double pY = p.getY();
            double pH = p.getHeading();

            xVariance += (pX - meanX) * (pX - meanX);
            yVariance += (pY - meanY) * (pY - meanY);
            headingVariance += (pH - meanHeading) * (pH - meanHeading);
        }

        xVariance /= size - 1;
        yVariance /= size - 1;
        headingVariance /= size - 1;

        return new double[]{xVariance, yVariance, headingVariance};
    }

    public void printTelemetry() {
        telemetry.addLine("TURRET");
        telemetry.addData("current state (ENUM): ", currentState);
        telemetry.addData("calculated distance (INCHES): ", getDistance());
        telemetry.addData("is PID in tolerance (BOOLEAN): ", isPIDInTolerance());

        if (robot.shooter.getQueuedShots() > 0) {
            dashTelemetry.addLine("TURRET");
            dashTelemetry.addData("vision setpoint (RADIANS): ", 0);
            dashTelemetry.addData("current vision (RADIANS): ", autoAim.getTargetYawDegrees());
            dashTelemetry.addData("encoder angle (ANGLE): ", currentAngle);
            dashTelemetry.addData("raw motor ticks (TICKS): ", motorEncoder.getPosition());
            dashTelemetry.addData("absolute encoder (ANGLE): ", normalizeToTurretRange(360 - ((absoluteEncoder.getVoltage() / 3.2 * 360 + ABSOLUTE_ENCODER_OFFSET) % 360) % 360));
            dashTelemetry.addData("target angle (ANGLE): ", targetAngle);
        }

        dashTelemetry.addLine("TURRET POSE (VISION/ODO)");
        dashTelemetry.addData("TURRET X (INCHES)", "%.4f", turretPos.getX());
        dashTelemetry.addData("TURRET Y (INCHES)", "%.4f", turretPos.getY());
        dashTelemetry.addData("Heading (DEGREES)", "%.1f", turretPos.getHeading());

        dashTelemetry.addLine("VISION RELOCALIZATION");
        dashTelemetry.addData("ROBOT VISION X (INCHES)", "%.4f", robotPoseFromVision.getX());
        dashTelemetry.addData("ROBOT VISION Y (INCHES)", "%.4f", robotPoseFromVision.getY());
        dashTelemetry.addData("Heading (DEGREES)", "%.2f", robotPoseFromVision.getHeading());
        dashTelemetry.addData("Pose (INCH, INCH, DEGREES)", "(%.4f, %.4f)  %.2f°", robotPoseFromVision.getX(), robotPoseFromVision.getY(), robotPoseFromVision.getHeading());
        dashTelemetry.addData("Vision Variance (INCH, INCH, DEGREES) (X,Y, H)", "(%.4f, %.4f)  %.2f", visionVariances[0], visionVariances[1], visionVariances[2]);
    }
}
