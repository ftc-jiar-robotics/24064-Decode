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
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.dashTelemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isBigTriangle;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isRed;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.decode.subsystem.RobotActions;
import org.firstinspires.ftc.teamcode.decode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.decode.util.Drawing;

@TeleOp(name = "Main TeleOp", group = "24064")
public class MainTeleOp extends LinearOpMode {
    GamepadEx gamepadEx1, gamepadEx2;

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
        robot.shooter.setGoalAlliance(isRed);

        waitForStart();

        if (!isRed) AUTO_END_POSE = AUTO_END_POSE.mirror().setHeading(isBigTriangle ? 270 : 90);

        robot.drivetrain.setStartingPose(AUTO_END_POSE);

        robot.drivetrain.update();
        robot.drivetrain.startTeleopDrive(true);

        boolean isSlowMode = false;

        while (opModeIsActive()) {
            robot.run();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            if (isSlowMode) robot.drivetrain.setMaxPowerScaling(0.7);
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
            double trigger2 = gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

            if (gamepadEx1.wasJustPressed(DPAD_LEFT)) isSlowMode = !isSlowMode;

            robot.intake.set(trigger1, false);
            if (Math.abs(trigger1) > 0) robot.shooter.setFeederIdle();

            if (gamepadEx2.wasJustPressed(RIGHT_BUMPER)) robot.shooter.toggleManual();

            if (robot.shooter.get() == Shooter.ShooterStates.MANUAL) {
                if (Math.abs(trigger2) > 0) robot.shooter.setTurretManual(trigger2);
                if (gamepadEx1.isDown(DPAD_UP)) robot.shooter.setHoodManual(0.5, true);
                if (gamepadEx1.isDown(DPAD_DOWN)) robot.shooter.setHoodManual(0.5, false);
            }

            if (gamepadEx1.wasJustPressed(A)) robot.actionScheduler.addAction(RobotActions.shootArtifacts(1));
            if (gamepadEx1.wasJustPressed(B)) robot.actionScheduler.addAction(RobotActions.shootArtifacts(3));
            if (gamepadEx1.wasJustPressed(X)) robot.shooter.clearQueueShots();

            if (gamepadEx1.wasJustPressed(DPAD_RIGHT)) robot.drivetrain.setPose(AUTO_END_POSE);


            // TODO test collisions and shooting
//            boolean isInFarTriangle = robot.shooterZoneChecker.checkRectangleTriangleIntersection(ShooterZoneChecker.farTriangle);
//            boolean isInCloseTriangle = robot.shooterZoneChecker.checkRectangleTriangleIntersection(ShooterZoneChecker.closeTriangle);
//
//            if (isInFarTriangle || isInCloseTriangle) {
//                robot.actionScheduler.addAction(RobotActions.shootArtifacts(1));
//            }

            robot.printTelemetry();
        }

        AUTO_END_POSE = null;
    }
}
