package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.INTAKE_NONE_MAX_CR;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.INTAKE_NONE_MIN_CR;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_FEEDER_COLOR_SENSOR;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.inTriangle;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isTelemetryOn;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;
import static org.firstinspires.ftc.teamcode.decode.util.ZoneChecker.closeTriangle;
import static org.firstinspires.ftc.teamcode.decode.util.ZoneChecker.farTriangle;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.HSV;
import org.firstinspires.ftc.teamcode.decode.sensor.ColorSensor;
import org.firstinspires.ftc.teamcode.decode.util.ActionScheduler;
import org.firstinspires.ftc.teamcode.decode.util.BulkReader;
import org.firstinspires.ftc.teamcode.decode.util.Drawing;
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
    public final VoltageSensor batteryVoltageSensor;
    public final LEDController ledController;

    public enum ArtifactColor {
        GREEN, PURPLE, NONE
    }




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
        ledController = new LEDController(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

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


    public static boolean isArtifactFound(ColorSensor colorSensor) {
        return !colorSensor.hsv.inRange(INTAKE_NONE_MIN_CR, INTAKE_NONE_MAX_CR);
    }

    public static Robot.ArtifactColor getColor(ColorSensor colorSensor, boolean isRev) {
        HSV minGreen = isRev ? Common.GREEN_MIN_REV : Common.GREEN_MIN_CR;
        HSV maxGreen = isRev ? Common.GREEN_MAX_REV : Common.GREEN_MAX_CR;
        HSV minPurple = isRev ? Common.PURPLE_MIN_REV : Common.PURPLE_MIN_CR;
        HSV maxPurple = isRev ? Common.PURPLE_MAX_REV : Common.PURPLE_MAX_CR;

        if (colorSensor.hsv.inRange(minGreen, maxGreen)) return Robot.ArtifactColor.GREEN;
        else if (colorSensor.hsv.inRange(minPurple, maxPurple)) return Robot.ArtifactColor.PURPLE;
        else return Robot.ArtifactColor.NONE;
    }
    // Runs all the necessary mechanisms
    public void run() {
        actionScheduler.run();
        shooter.run();
        intake.run();
        update();
    }

    public void update() {
        drivetrain.update();
        LoopUtil.updateLoopCount();
        zoneChecker.setRectangle(drivetrain.getPose().getX(), drivetrain.getPose().getY(), drivetrain.getPose().getHeading());
        Common.inTriangle = robot.zoneChecker.checkRectangleTriangleIntersection(farTriangle) || robot.zoneChecker.checkRectangleTriangleIntersection(closeTriangle);

        int ballCount = 0;

        if (!inTriangle && shooter.getQueuedShots() <= 0) {
                switch (intake.getColor()) {
                    case NONE: break;
                    case GREEN:
                    case PURPLE:
                        ballCount = 3;
                }


            ledController.update(ballCount);
        } else ledController.showShooterTolerance();


        readSensors();
    }

    // Prints data on the driver hub for debugging and other uses
    public void printTelemetry() {
        if (isTelemetryOn) {
            shooter.printTelemetry();
            intake.printTelemetry();
        }
        Common.telemetry.addData("robot x (DOUBLE): ", drivetrain.getPose().getX());
        Common.telemetry.addData("robot y (DOUBLE): ", drivetrain.getPose().getY());
        Common.telemetry.addData("robot heading (ANGLE): ", Math.toDegrees(drivetrain.getPose().getHeading()));
        Common.telemetry.addData("robot max power: ", robot.drivetrain.getMaxPowerScaling());
        Common.telemetry.addData("loop time (LOOPS): ", LoopUtil.getLoopTimeInHertz());

        Drawing.drawRobot(robot.shooter.getPredictedPose(), new Style("", "#FF0000", 2.0));
        Drawing.drawDebug(drivetrain);

        Common.telemetry.update();
        Common.dashTelemetry.update();
    }
}
