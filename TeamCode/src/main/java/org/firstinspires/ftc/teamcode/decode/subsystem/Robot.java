package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.FAR_DISTANCE;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.LOCALIZATION_X;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.LOCALIZATION_Y;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.MID_DISTANCE;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.MIN_MOVEMENT_SPEED;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isRed;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isTelemetryOn;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;

import android.util.Log;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.HSV;
import org.firstinspires.ftc.teamcode.decode.sensor.ColorSensor;
import org.firstinspires.ftc.teamcode.decode.util.ActionScheduler;
import org.firstinspires.ftc.teamcode.decode.util.ArduCam;
import org.firstinspires.ftc.teamcode.decode.util.BulkReader;
import org.firstinspires.ftc.teamcode.decode.util.Drawing;
import org.firstinspires.ftc.teamcode.decode.util.LimelightEx;
import org.firstinspires.ftc.teamcode.decode.util.LoopUtil;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Configurable
public final class Robot {
    public final Follower drivetrain;
    public final BulkReader bulkReader;
    public final ActionScheduler actionScheduler;
    public final GateOpener gateOpener;
    public final Shooter shooter;
    public final Intake intake;
    private final VoltageSensor batteryVoltageSensor;
//    public final LEDController ledController;

    public static double MAX_STALENESS = 1e9;

    public static double
            MAX_VARIANCE_X = 0.7,
            MAX_VARIANCE_Y = 0.7;

    public LimelightEx limelight;
    public ArduCam arducam;

    public boolean hasArduCamRelocalized = false;

    public final boolean isAuto;
    public boolean isFar, isMid;

    private double voltage;

    public enum ArtifactColor {
        GREEN, PURPLE, NONE
    }

    private boolean isRobotMoving = false;

    /**
     * Constructor used in teleOp classes that makes the current pose2d, 0
     * @param hardwareMap A constant map that holds all the parts for config in code
     */
    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, false);
    }

    /**
     * Constructor for instantiating a new 'robot' class
     * @param hardwareMap: A constant map that holds all the parts for config in code
     */
    public Robot(HardwareMap hardwareMap, boolean isAuto) {
        this.isAuto = isAuto;

        if (isAuto) {
            Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
            limelight = new LimelightEx(limelight3A);
        }

        drivetrain = Constants.createFollower(hardwareMap);
        bulkReader = new BulkReader(hardwareMap);
        actionScheduler = new ActionScheduler();
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        gateOpener = new GateOpener(hardwareMap);
//        ledController = new LEDController(hardwareMap);
        if (!isAuto) {
            arducam = new ArduCam(hardwareMap, "arducam");
//            gateOpener.set(GateOpener.GateOpenerStates.AUTOMATIC,true);
        }


//        ledController.ensureInitialized();

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        Drawing.init();

        try {
            drivetrain.getPoseTracker().resetIMU();
        } catch (InterruptedException ignored) {}

        shooter.applyOffsets();
    }

    public boolean isRobotMoving() {
        return isRobotMoving;
    }



    // Reads all the necessary sensors (including battery volt.) in one bulk read
    public void readSensors() {
        bulkReader.bulkRead();
        voltage = batteryVoltageSensor.getVoltage();
    }

    public double getVoltage(){
        return voltage;
    }

    // Runs all the necessary mechanisms
    public void run() {
        update();
        drivetrain.update();
        shooter.run();
        intake.run();
        actionScheduler.run();
        gateOpener.run();
    }

    public void update() {
        readSensors();
        isRobotMoving = isRobotMoving(MIN_MOVEMENT_SPEED);

        isFar = shooter.turret.getDistance() > FAR_DISTANCE;
        isMid = !isFar && shooter.turret.getDistance() > MID_DISTANCE;

        if (!isAuto) arducam.detectTarget();

        Common.inTriangle = LaunchZone.getCurrentZone(drivetrain.getPose()) != LaunchZone.NONE;

//        ledController.update();
        LoopUtil.updateLoopCount();
    }

    public boolean isRobotMoving(double minMovementSpeed) {
        return Math.hypot(drivetrain.getVelocity().getXComponent(), drivetrain.getVelocity().getYComponent()) > minMovementSpeed;
    }

    public void relocalizeWithWall() {
        if (!isRed) {
            LOCALIZATION_X = 136.52;
        } else {
            LOCALIZATION_X = 7.48;
        }
        LOCALIZATION_Y = 10.039;

        drivetrain.setPose(new Pose(LOCALIZATION_X, LOCALIZATION_Y, Math.toRadians(270)));
    }

    public void relocalizeWithArdu() {
        relocalizeWithArdu(false);
    }

    public void relocalizeWithArdu(boolean override) {
        if (!isAuto && !robot.isFar) {
            Pose arduRobotPose = arducam.getTurretPosePedro();

            hasArduCamRelocalized = arduRobotPose != null && arducam.getStaleness() < MAX_STALENESS && arducam.getVariances()[0] < MAX_VARIANCE_X && arducam.getVariances()[1] < MAX_VARIANCE_Y && !isRobotMoving;

            if (arduRobotPose != null && (hasArduCamRelocalized || override)) {
                drivetrain.setPose(new Pose(arduRobotPose.getX(), arduRobotPose.getY(), drivetrain.getHeading()));
            }
        }
    }

    // Prints data on the driver hub for debugging and other uses
    public void printTelemetry() {
        if (isTelemetryOn) {
            shooter.printTelemetry();
            intake.printTelemetry();
            gateOpener.printTelemetry();
            if (isAuto) limelight.printTelemetry();
            if (!isAuto) arducam.printTelemetry();

            Common.telemetry.addData("robot x (DOUBLE): ", drivetrain.getPose().getX());
            Common.telemetry.addData("robot y (DOUBLE): ", drivetrain.getPose().getY());
            Common.telemetry.addData("robot heading (ANGLE): ", Math.toDegrees(drivetrain.getPose().getHeading()));
            Common.telemetry.addData("robot max power: ", drivetrain.getMaxPowerScaling());
            Drawing.drawDebug(drivetrain);
            Drawing.drawRobot(LaunchZone.getInterceptOrClosestPoint());

            Common.telemetry.update();
            Common.dashTelemetry.update();
        }

        Log.d("loop time (LOOPS): ", "" + LoopUtil.getLoopTimeInHertz());

    }
}
