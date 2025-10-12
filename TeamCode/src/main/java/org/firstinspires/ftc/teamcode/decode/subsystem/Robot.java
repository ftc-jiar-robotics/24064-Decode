package org.firstinspires.ftc.teamcode.decode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.decode.util.ActionScheduler;
import org.firstinspires.ftc.teamcode.decode.util.BulkReader;
import org.firstinspires.ftc.teamcode.decode.util.LoopUtil;
import org.firstinspires.ftc.teamcode.decode.util.ZoneChecker;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Configurable
public final class Robot {
    public final Follower drivetrain;
    public final BulkReader bulkReader;
    public final ActionScheduler actionScheduler;
    public final Shooter shooter;
    public final Intake intake;
    public final ZoneChecker zoneChecker;

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
        drivetrain = Constants.createFollower(hardwareMap);
        bulkReader = new BulkReader(hardwareMap);
        actionScheduler = new ActionScheduler();
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        zoneChecker = new ZoneChecker();
    }

    // Reads all the necessary sensors (including battery volt.) in one bulk read
    public void readSensors() {
        bulkReader.bulkRead();
    }

    // Runs all the necessary mechanisms
    public void run() {
        actionScheduler.run();
        shooter.run();
        intake.run();
        LoopUtil.updateLoopCount();
        update();
    }

    public void update() {
        drivetrain.update();
        zoneChecker.setRectangle(drivetrain.getPose().getX(), drivetrain.getPose().getY(), drivetrain.getPose().getHeading());
        readSensors();
    }

    // Prints data on the driver hub for debugging and other uses
    public void printTelemetry() {
        shooter.printTelemetry();
        Common.telemetry.addData("robot x: ", drivetrain.getPose().getX());
        Common.telemetry.addData("robot y: ", drivetrain.getPose().getY());
        Common.telemetry.addData("robot heading: ", Math.toDegrees(drivetrain.getPose().getHeading()));

        Common.graph.update();
        Common.telemetry.update();
        Common.dashTelemetry.update();
    }
}
