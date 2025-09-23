package org.firstinspires.ftc.teamcode.decode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.decode.util.ActionScheduler;
import org.firstinspires.ftc.teamcode.decode.util.BulkReader;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Config
public final class Robot {
    public final Follower drivetrain;
    public final BulkReader bulkReader;
    public final ActionScheduler actionScheduler;
    public final Shooter shooter;

    public enum State {
        NEUTRAL
    }

    State currentState = State.NEUTRAL;

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
    }

    // Reads all the necessary sensors (including battery volt.) in one bulk read
    public void readSensors() {
        bulkReader.bulkRead();
    }

    // Runs all the necessary mechanisms
    public void run() {

        actionScheduler.run();
        update();
    }

    public void update() {
        drivetrain.update();
    }

    // Prints data on the driver hub for debugging and other uses
    public void printTelemetry() {
    }

    public State getCurrentState() {
        return currentState;
    }
    public void setCurrentState(State state) {
        this.currentState = state;
    }
}
