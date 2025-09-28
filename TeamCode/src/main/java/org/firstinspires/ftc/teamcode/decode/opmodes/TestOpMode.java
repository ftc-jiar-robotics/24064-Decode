package org.firstinspires.ftc.teamcode.decode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.subsystem.Feeder;
import org.firstinspires.ftc.teamcode.decode.subsystem.Flywheel;
import org.firstinspires.ftc.teamcode.decode.subsystem.Hood;
import org.firstinspires.ftc.teamcode.decode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.decode.subsystem.Shooter;

@TeleOp(name = "Test Opmode", group = "24064")
public class TestOpMode extends LinearOpMode {
    private Flywheel flywheel;
    private Hood hood;
    private Feeder feeder;
    private Intake intake;

    private GamepadEx gamepadEx1;

    @Override
    public void runOpMode() throws InterruptedException {
        gamepadEx1 = new GamepadEx(gamepad1);

        feeder = new Feeder(hardwareMap);
        hood = new Hood(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        intake = new Intake(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            gamepadEx1.readButtons();

            double rTrigger = gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            for (MotorEx m : flywheel.motorGroup) {
                m.set(rTrigger);
            }

            double currentHoodAngle = 0;

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.X)) {
                currentHoodAngle += 30;
                hood.set(currentHoodAngle);
            }

            if (gamepadEx1.wasJustPressed(Y)) {
                currentHoodAngle -= 30;
                hood.set(currentHoodAngle);
            }


            intake.set(gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), true);

//            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.X)) {
//                feeder.set(new Feeder.FeederControl(false, false, Feeder.FeederStates.RUNNING));
//            }
//            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.Y)) {
//                feeder.set(new Feeder.FeederControl(true, false, Feeder.FeederStates.RUNNING));
//            }
//            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.B)) {
//                feeder.set(new Feeder.FeederControl(false, false, Feeder.FeederStates.OUTTAKING));
//            }
//            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) {
//                feeder.set(new Feeder.FeederControl(false, false, Feeder.FeederStates.OFF));
//            }

            double leftJoystick = gamepadEx1.getLeftY();
            double rightJoystick = gamepadEx1.getRightY();

            feeder.feederFront.setPower(leftJoystick);
            feeder.feederBack.setPower(rightJoystick);

            hood.run();
            intake.run();

            hood.printTelemetry();
            flywheel.printTelemetry();
            feeder.printTelemetry();
            Common.telemetry.update();
        }
    }
}
