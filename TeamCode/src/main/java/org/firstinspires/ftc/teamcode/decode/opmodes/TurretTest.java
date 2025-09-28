package org.firstinspires.ftc.teamcode.decode.opmodes;

import android.provider.Settings;
import android.provider.Telephony;
import android.util.Log;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.subsystem.Turret;

@TeleOp(name = "turret opmode", group = "24064")
public class TurretTest extends LinearOpMode {
    private GamepadEx gamepadEx1;
    private Turret turret;

    @Override
    public void runOpMode() {
        gamepadEx1 = new GamepadEx(gamepad1);

        turret = new Turret(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            gamepadEx1.readButtons();

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) turret.set(0.0);
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.B)) turret.set(90.0);
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.X)) turret.set(180.0);

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) turret.setManualPower(gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

            turret.run();
            turret.printTelemetry();
            Common.telemetry.update();
        }
    }
}
