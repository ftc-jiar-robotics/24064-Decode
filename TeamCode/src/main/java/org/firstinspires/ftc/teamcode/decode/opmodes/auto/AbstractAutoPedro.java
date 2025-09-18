package org.firstinspires.ftc.teamcode.decode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.subsystem.Robot;


@Disabled
public abstract class AbstractAutoPedro extends LinearOpMode {

    protected final void update() {
        robot.readSensors();
        robot.update();
    }

    @Override
    public final void runOpMode() {
        robot = new Robot(hardwareMap);

        robot.actionScheduler.setUpdate(this::update);

        onInit();
        configure();

        if (isStopRequested()) return;

        waitForStart();

        resetRuntime();
        robot.drivetrain.setPose(getStartPose());

        onRun();
        Common.AUTO_END_POSE = robot.drivetrain.getPose();
    }

    protected void onInit() {
        robot.drivetrain.setPose(getStartPose());
    }
    protected void configure() {}
    protected abstract Pose getStartPose();
    protected abstract void onRun();
}