package org.firstinspires.ftc.teamcode.decode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.AUTO_END_POSE;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.SLOW_MODE;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.dashTelemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isBigTriangle;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isHoodManual;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isRed;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isSlowMode;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;
import static org.firstinspires.ftc.teamcode.decode.util.ZoneChecker.closeTriangle;
import static org.firstinspires.ftc.teamcode.decode.util.ZoneChecker.farTriangle;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.subsystem.Feeder;
import org.firstinspires.ftc.teamcode.decode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.decode.subsystem.RobotActions;
import org.firstinspires.ftc.teamcode.decode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.decode.util.Drawing;
import org.firstinspires.ftc.teamcode.decode.util.LoopUtil;

@TeleOp(name = "Main TeleOp", group = "24064")
public class MainTeleOp extends LinearOpMode {
    GamepadEx gamepadEx1, gamepadEx2;

    boolean lastInTriangle = false;

    private int storedShots = 0;

    @Override
    public void runOpMode() {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        Common.dashTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (AUTO_END_POSE == null) {
            while (opModeInInit()) {
                gamepadEx1.readButtons();

                dashTelemetry.addLine("PRESS X TO TOGGLE SIDES");
                dashTelemetry.addLine("PRESS O TO TOGGLE TRIANGLE LOCATION");
                dashTelemetry.addData("IS RED: ", isRed);
                if (gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) Common.isRed = !isRed;
                dashTelemetry.addData("IS BIG TRIANGLE: ", isBigTriangle);
                if (gamepadEx1.wasJustPressed(GamepadKeys.Button.B)) isBigTriangle = !isBigTriangle;

                AUTO_END_POSE = isBigTriangle ? new Pose(113.5, 135.5, Math.toRadians(270)) : new Pose(88.6, 7.25, Math.toRadians(90));

                dashTelemetry.update();
            }
        }

        robot = new Robot(hardwareMap);
        robot.shooter.setGoalAlliance();

        waitForStart();

        if (!isRed) AUTO_END_POSE = AUTO_END_POSE.mirror().setHeading(isBigTriangle ? (3.0 * Math.PI) / 2.0 : Math.PI / 2.0);

        robot.drivetrain.setStartingPose(AUTO_END_POSE);

        robot.drivetrain.update();
        robot.drivetrain.startTeleopDrive(true);

        while (opModeIsActive()) {
            robot.run();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            if (isSlowMode) robot.drivetrain.setMaxPowerScaling(SLOW_MODE);
            else robot.drivetrain.setMaxPowerScaling(1);

            if (isRed) {
                robot.drivetrain.setTeleOpDrive(
                        gamepadEx1.getLeftY(),
                        -gamepadEx1.getLeftX(),
                        -gamepadEx1.getRightX(),
                        false
                );
            } else {
                robot.drivetrain.setTeleOpDrive(
                        -gamepadEx1.getLeftY(),
                        gamepadEx1.getLeftX(),
                        -gamepadEx1.getRightX(),
                        false
                );
            }

            double trigger1 = gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

            isSlowMode = robot.shooter.get() == Shooter.ShooterStates.RUNNING || robot.shooter.get() == Shooter.ShooterStates.PREPPING;

            robot.intake.set(trigger1, false);
            robot.shooter.setFeederIdle(Math.abs(trigger1) > 0);

            if (isHoodManual) {
                if (gamepadEx1.isDown(DPAD_UP)) robot.shooter.setHoodManual(0.5, true);
                if (gamepadEx1.isDown(DPAD_DOWN)) robot.shooter.setHoodManual(0.5, false);
            }

            if (gamepadEx1.wasJustPressed(A)) robot.actionScheduler.addAction(RobotActions.shootArtifacts(1));
            if (gamepadEx1.wasJustPressed(B)) robot.actionScheduler.addAction(RobotActions.shootArtifacts(3));
            if (gamepadEx1.wasJustPressed(X)) robot.shooter.clearQueueShots();
            if (gamepadEx1.wasJustPressed(DPAD_RIGHT)) robot.drivetrain.setPose(AUTO_END_POSE);


            // if Intake see's one queue 3 shots,
            // However if feeder see's smthn but intake doesn't then queue 1 shot
            // TODO Make this an action
            if (Common.inTriangle) {
                Robot.ArtifactColor feederColor = robot.shooter.getColor();
                if (feederColor != Robot.ArtifactColor.NONE) {
                    int shots = (robot.intake.getColor() != Robot.ArtifactColor.NONE) ? 3 : 1;
                    if (robot.shooter.getQueuedShots() == 0)
                        robot.shooter.setQueuedShots(storedShots == 0 ? shots : storedShots);

                }
                if ((robot.shooter.get() == Shooter.ShooterStates.PREPPING || robot.shooter.get() == Shooter.ShooterStates.RUNNING) && robot.shooter.getQueuedShots() > 0)
                    robot.intake.set(0.85, true);
            } else if (lastInTriangle) {
                storedShots = robot.shooter.getQueuedShots();
                robot.shooter.clearQueueShots();
            }

            lastInTriangle = Common.inTriangle;

            robot.printTelemetry();
        }

        AUTO_END_POSE = null;
    }
}
