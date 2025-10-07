package org.firstinspires.ftc.teamcode.decode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.subsystem.Hood;

@Configurable
@TeleOp(name = "hood resetting opmode", group = "24064")
public class HoodResetttingOpMode extends LinearOpMode {
    public Hood hood;
    public GamepadEx gamepadEx1;

    @Override
    public void runOpMode() {
        hood = new Hood(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepadEx1.wasJustPressed(A)) hood.setPhysicalMax();
            if (gamepadEx1.wasJustPressed(B)) hood.set(hood.MAX);
            if (gamepadEx1.wasJustPressed(X)) hood.set(hood.MIN);

            hood.run();
            hood.printTelemetry();
            Common.telemetry.update();
        }
    }
}
