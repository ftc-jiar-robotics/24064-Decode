package org.firstinspires.ftc.teamcode.decode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.decode.subsystem.RobotActions;

@TeleOp(name = "Main Teleop", group = "24064")
public class MainTeleop extends LinearOpMode {
    GamepadEx gamepadEx1, gamepadEx2;

    @Override
    public void runOpMode() {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        robot = new Robot(hardwareMap);

        waitForStart();

        // TODO replace pose w/ pose at the end of auton
        robot.drivetrain.setStartingPose(new Pose(70, 136.8, Math.toRadians(90)));
        robot.drivetrain.update();
        robot.drivetrain.startTeleopDrive(true);


        while (opModeIsActive()) {
            robot.run();

            robot.drivetrain.setTeleOpDrive(
                    gamepadEx1.getLeftY(),
                    -gamepadEx1.getLeftX(),
                    -gamepadEx1.getRightX(),
                    false
            );

            double trigger = gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            if (Math.abs(trigger) > 0) robot.intake.set(trigger, false);
            else robot.intake.set(0.0, false);

            // TODO automatically detect shooting zone and shoot
            if (gamepadEx1.wasJustReleased(A)) {
                robot.actionScheduler.addAction(RobotActions.shootArtifacts(1));
            }

            robot.printTelemetry();
        }
    }
}
