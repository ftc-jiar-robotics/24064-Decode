package org.firstinspires.ftc.teamcode.decode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.decode.subsystem.RobotActions;
import org.firstinspires.ftc.teamcode.decode.subsystem.Shooter;

@TeleOp(name = "Main TeleOp", group = "24064")
public class MainTeleOp extends LinearOpMode {
    GamepadEx gamepadEx1, gamepadEx2;

    @Override
    public void runOpMode() {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        robot = new Robot(hardwareMap);

        Common.dashTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        // TODO replace pose w/ pose at the end of auton
        robot.drivetrain.setStartingPose(new Pose(70, 136.8, Math.toRadians(90)));
        robot.drivetrain.update();
        robot.drivetrain.startTeleopDrive(true);


        while (opModeIsActive()) {
            robot.run();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            robot.drivetrain.setTeleOpDrive(
                    gamepadEx1.getLeftY(),
                    -gamepadEx1.getLeftX(),
                    -gamepadEx1.getRightX(),
                    false
            );

            double trigger1 = gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            if (Math.abs(trigger1) > 0) robot.intake.set(trigger1, false);
            else robot.intake.set(0.0, false);

            // TODO automatically detect shooting zone and shoot
            if (gamepadEx1.wasJustPressed(A)) {
                robot.actionScheduler.addAction(RobotActions.shootArtifacts(1));
            }

            if (gamepadEx1.wasJustPressed(B)) {
                robot.actionScheduler.addAction(RobotActions.shootArtifacts(3));
            }

            if (gamepadEx1.wasJustPressed(X)) {
                robot.shooter.clearQueueShots();
            }

            if (gamepadEx1.isDown(DPAD_UP)) {
                robot.shooter.hood.set(robot.shooter.hood.get() + .5);
            }
            if (gamepadEx1.isDown(DPAD_DOWN)) {
                robot.shooter.hood.set(robot.shooter.hood.get() - .5);
            }

            robot.printTelemetry();
        }
    }
}
