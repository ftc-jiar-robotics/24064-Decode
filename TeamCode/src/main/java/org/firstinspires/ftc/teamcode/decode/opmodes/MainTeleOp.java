package org.firstinspires.ftc.teamcode.decode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_STICK_BUTTON;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_STICK_BUTTON;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.AUTO_END_POSE;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.SLOW_MODE;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.dashTelemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isBigTriangle;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isFlywheelManual;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isFuturePoseOn;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isHoodManual;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isRed;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isSlowMode;
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

    boolean lastInTriangle = false;

    private int storedShots = 0;

    @Override
    public void runOpMode() {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        Common.dashTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        isFuturePoseOn = true;

        if (AUTO_END_POSE == null) {
            while (opModeInInit()) {
                gamepadEx1.readButtons();

                dashTelemetry.addLine("PRESS X TO TOGGLE SIDES");
                dashTelemetry.addLine("PRESS O TO TOGGLE TRIANGLE LOCATION");
                dashTelemetry.addData("IS RED: ", isRed);
                if (gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) Common.isRed = !isRed;
                dashTelemetry.addData("IS BIG TRIANGLE: ", isBigTriangle);
                if (gamepadEx1.wasJustPressed(GamepadKeys.Button.B)) isBigTriangle = !isBigTriangle;
                dashTelemetry.update();
            }
            AUTO_END_POSE = isRed ? (isBigTriangle ? Common.RED_BIG_TRIANGLE : Common.RED_SMALL_TRIANGLE) : (isBigTriangle ? Common.BLUE_BIG_TRIANGLE : Common.BLUE_SMALL_TRIANGLE);

        }

        robot = new Robot(hardwareMap);
        robot.shooter.setGoalAlliance();

        waitForStart();

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

            if (isHoodManual) {
                if (gamepadEx1.isDown(DPAD_UP)) robot.shooter.setHoodManual(0.5, true);
                if (gamepadEx1.isDown(DPAD_DOWN)) robot.shooter.setHoodManual(0.5, false);
            }

            if (isFlywheelManual) {
                if (gamepadEx1.isDown(DPAD_RIGHT)) robot.shooter.incrementFlywheelRPM(5, true);
                if (gamepadEx1.isDown(DPAD_LEFT)) robot.shooter.incrementFlywheelRPM(5, false);

            }

            if (gamepadEx1.wasJustPressed(A)) robot.actionScheduler.addAction(RobotActions.shootArtifacts(1));
            if (gamepadEx1.wasJustPressed(B)) robot.actionScheduler.addAction(RobotActions.shootArtifacts(3));
            if (gamepadEx1.wasJustPressed(X)) robot.shooter.clearQueueShots();
            if (gamepadEx1.isDown(RIGHT_STICK_BUTTON) && gamepadEx1.isDown(LEFT_STICK_BUTTON)) robot.drivetrain.setPose(AUTO_END_POSE);
            if (gamepadEx1.wasJustPressed(RIGHT_BUMPER)) {
                Pose pose = robot.drivetrain.getPose();
                double snappedHeadingRad = Math.toRadians(
                        Math.round(Math.toDegrees(pose.getHeading()) / 90.0) * 90.0
                );
                robot.drivetrain.setPose(new Pose(pose.getX(), pose.getY(), snappedHeadingRad));
            }


            // if Intake see's one queue 3 shots,
            // However if feeder see's smthn but intake doesn't then queue 1 shot
            if (Common.inTriangle) {
                boolean isBallPresent = robot.shooter.isBallPresent();
                if (isBallPresent) {
                    int shots = 3;
                    if (robot.shooter.getQueuedShots() == 0) {
                        robot.shooter.setQueuedShots(storedShots == 0 ? shots : storedShots);
                        storedShots = 0;
                    }
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
        robot.shooter.closeAutoAim();
    }
}
