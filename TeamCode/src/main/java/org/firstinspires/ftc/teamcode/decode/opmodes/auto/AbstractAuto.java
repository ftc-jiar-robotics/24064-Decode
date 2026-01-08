package org.firstinspires.ftc.teamcode.decode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.dashTelemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isRed;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.util.Logging.MatchReplayLogger;
import org.firstinspires.ftc.teamcode.decode.subsystem.Robot;


@Disabled
public abstract class AbstractAuto extends LinearOpMode {
    protected Follower f;

    protected void update() {
        robot.update();
        robot.drivetrain.update();
        robot.shooter.run();
        robot.intake.run();
        robot.printTelemetry();

        double t = getRuntime();
        if (lastLogSec < 0 || (t - lastLogSec) >= LOG_PERIOD_SEC) {
            lastLogSec = t;
            MatchReplayLogger.get().logRobot(robot, t);
        }

    }
    private double lastLogSec = -1;
    private static final double LOG_PERIOD_SEC = 0.05; // 20 Hz

    @Override
    public final void runOpMode() {
        Common.TURRET_ENC_OFFSET = Double.POSITIVE_INFINITY;

        robot = new Robot(hardwareMap, true);

        robot.actionScheduler.setUpdate(this::update);

        configure();

        onInit();

        if (isStopRequested()) return;

        waitForStart();

        MatchReplayLogger.get().startAutonomous(getClass().getSimpleName());
        MatchReplayLogger.get().event(0.0, "AUTO_SELECTED", getClass().getSimpleName());
        try {
            robot.drivetrain.setPose(getStartPose());
            onRun();

            robot.actionScheduler.addAction(new SleepAction(2));
            robot.actionScheduler.runBlocking();

            Common.AUTO_END_POSE = robot.drivetrain.getPose();
            Common.TURRET_ENC_OFFSET = robot.shooter.getTurretAngle();
        } finally {
            // this runs even if you stop early
            MatchReplayLogger.get().stopAutonomous(getRuntime());
            robot.limelight.getLimelight().close(); // move your close here too
        }

        resetRuntime();
        robot.drivetrain.setPose(getStartPose());
        onRun();
        robot.actionScheduler.addAction(new SleepAction(2));
        robot.actionScheduler.runBlocking();
        Common.AUTO_END_POSE = robot.drivetrain.getPose();
        Common.TURRET_ENC_OFFSET = robot.shooter.getTurretAngle();
        robot.limelight.getLimelight().close();
    }

    protected void onInit() {
        robot.drivetrain.setPose(getStartPose());
    }

    protected void configure() {
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        Common.dashTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (opModeInInit()) {
            gamepadEx1.readButtons();

            dashTelemetry.addLine("PRESS A TO TOGGLE SIDES");
            dashTelemetry.addData("IS RED: ", Common.isRed);
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) Common.isRed = !Common.isRed;

            dashTelemetry.update();
        }
    }
    protected abstract Pose getStartPose();
    protected abstract void onRun();
}