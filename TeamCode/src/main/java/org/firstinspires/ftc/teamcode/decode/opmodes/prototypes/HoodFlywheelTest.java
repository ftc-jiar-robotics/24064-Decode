package org.firstinspires.ftc.teamcode.decode.opmodes.prototypes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.subsystem.Flywheel;
import org.firstinspires.ftc.teamcode.decode.subsystem.Hood;

@TeleOp(name = "hood flywheel test", group = "prototypes")
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

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) flywheel.set(Flywheel.FlyWheelStates.RUNNING);
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.B)) flywheel.set(Flywheel.FlyWheelStates.ARMING);

            double currentHoodAngle = 0;

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) currentHoodAngle += 5;

            hood.set(currentHoodAngle);
            flywheel.run();
            hood.run();

            hood.printTelemetry();
            flywheel.printTelemetry();
            Common.telemetry.update();
        }
    }
}
