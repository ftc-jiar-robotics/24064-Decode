package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isTelemetryOn;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Flywheel.MIN_MOVEMENT_SPEED;

import android.util.Log;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.decode.util.ActionScheduler;
import org.firstinspires.ftc.teamcode.decode.util.BulkReader;
import org.firstinspires.ftc.teamcode.decode.util.Drawing;
import org.firstinspires.ftc.teamcode.decode.util.LoopUtil;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Configurable
public final class Robot {
    public final Follower drivetrain;
    public final BulkReader bulkReader;
    public final ActionScheduler actionScheduler;
    public final Shooter shooter;
    public final Intake intake;
    public final VoltageSensor batteryVoltageSensor;

    public final boolean isAuto;
    public boolean isFar;

    public boolean isMid;

    public static double
        FAR_DISTANCE = 120,
        MID_DISTANCE = 100;

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

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        drivetrain = Constants.createFollower(hardwareMap);
        bulkReader = new BulkReader(hardwareMap);
        actionScheduler = new ActionScheduler();
        shooter = new Shooter(hardwareMap, drivetrain, batteryVoltageSensor);
        intake = new Intake(hardwareMap);

        Drawing.init();

        try {
            drivetrain.getPoseTracker().resetIMU();
        } catch (InterruptedException ignored) {}

        shooter.applyOffsets();
    }

    // Reads all the necessary sensors (including battery volt.) in one bulk read
    public void readSensors() {
        bulkReader.bulkRead();
    }

    // Runs all the necessary mechanisms
    public void run() {
        update();
        drivetrain.update();

        shooter.run();
        intake.run();
        actionScheduler.run();
    }

    public void update() {
        readSensors();
        isFar = shooter.turret.getDistance() > FAR_DISTANCE;
        isMid = !isFar && shooter.turret.getDistance() > MID_DISTANCE;

        shooter.flywheel.movingToFarZone = drivetrain.getPose().getY() < 40 &&
                drivetrain.getVelocity().getYComponent() < -0.3;

        shooter.flywheel.isMoving = Math.hypot(drivetrain.getVelocity().getXComponent(), drivetrain.getVelocity().getYComponent()) > MIN_MOVEMENT_SPEED;;

        Common.inTriangle = LaunchZone.getCurrentZone(drivetrain.getPose()) != LaunchZone.NONE;

        LoopUtil.updateLoopCount();
    }


    // Prints data on the driver hub for debugging and other uses
    public void printTelemetry() {
        if (isTelemetryOn) {
            shooter.printTelemetry();
            intake.printTelemetry();

            Common.telemetry.addData("robot x (DOUBLE): ", drivetrain.getPose().getX());
            Common.telemetry.addData("robot y (DOUBLE): ", drivetrain.getPose().getY());
            Common.telemetry.addData("robot heading (ANGLE): ", Math.toDegrees(drivetrain.getPose().getHeading()));
            Common.telemetry.addData("robot max power: ", drivetrain.getMaxPowerScaling());
            Drawing.drawDebug(drivetrain);

            Common.telemetry.update();
            Common.dashTelemetry.update();
        } else {
            Log.d("loop time (LOOPS): ", "" + LoopUtil.getLoopTimeInHertz());
        }
    }
}