package org.firstinspires.ftc.teamcode.decode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;

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

                AUTO_END_POSE = isBigTriangle ? new Pose(112.75, 136.5, Math.toRadians(270)) : new Pose(88.6, 7.25, Math.toRadians(90));

                dashTelemetry.update();
            }
        }

        robot = new Robot(hardwareMap);

        waitForStart();

        if (!isRed) AUTO_END_POSE = AUTO_END_POSE.mirror().setHeading(isBigTriangle ? 270 : 90);

        // TODO replace pose w/ pose at the end of auton

        robot.drivetrain.setStartingPose(AUTO_END_POSE);

        robot.drivetrain.update();
        robot.drivetrain.startTeleopDrive(true);

        while (opModeIsActive()) {
            robot.run();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

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

            robot.intake.set(trigger1, false);
            robot.shooter.setFeederManual(trigger1 * 0.7, -Math.abs(trigger1));

            if (gamepadEx1.wasJustPressed(A)) {
                robot.actionScheduler.addAction(RobotActions.shootArtifacts(1));
            }
            if (gamepadEx1.wasJustPressed(DPAD_UP)) {
                robot.drivetrain.setPose(AUTO_END_POSE);
            }
            if (gamepadEx1.wasJustPressed(B)) {
                robot.actionScheduler.addAction(RobotActions.shootArtifacts(3));
            }

            if (gamepadEx1.wasJustPressed(X)) {
                robot.shooter.clearQueueShots();
            }

//            boolean isInFarTriangle = robot.shooterZoneChecker.checkRectangleTriangleIntersection(ShooterZoneChecker.farTriangle);
//            boolean isInCloseTriangle = robot.shooterZoneChecker.checkRectangleTriangleIntersection(ShooterZoneChecker.closeTriangle);
//
//            if (isInFarTriangle || isInCloseTriangle) {
//                robot.actionScheduler.addAction(RobotActions.shootArtifacts(1));
//            }

//            if (gamepadEx1.isDown(DPAD_UP)) {
//                robot.shooter.hood.set(robot.shooter.hood.get() + .5);
//            }
//            if (gamepadEx1.isDown(DPAD_DOWN)) {
//                robot.shooter.hood.set(robot.shooter.hood.get() - .5);
//            }

            robot.printTelemetry();
        }

        AUTO_END_POSE = null;
    }
}
