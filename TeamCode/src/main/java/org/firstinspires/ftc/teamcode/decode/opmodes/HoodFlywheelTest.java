package org.firstinspires.ftc.teamcode.decode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.subsystem.Flywheel;
import org.firstinspires.ftc.teamcode.decode.subsystem.Hood;

@TeleOp(name = "Hood Flywheel Test", group = "24064")
public class HoodFlywheelTest extends LinearOpMode {
    private GamepadEx gamepadEx1;
    private Hood hood;
    private Flywheel flywheel;

    @Override
    public void runOpMode() {
        gamepadEx1 = new GamepadEx(gamepad1);

        hood = new Hood(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            gamepadEx1.readButtons();

            double rTrigger = gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            for (MotorEx m : flywheel.motorGroup) {
                m.set(rTrigger);
            }

            double currentHoodAngle = 0;

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) currentHoodAngle += 5;

            hood.set(currentHoodAngle);
            hood.run();

            hood.printTelemetry();
            flywheel.printTelemetry();
            Common.telemetry.update();
        }
    }
}
