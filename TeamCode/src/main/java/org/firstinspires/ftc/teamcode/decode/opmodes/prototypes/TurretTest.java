package org.firstinspires.ftc.teamcode.decode.opmodes.prototypes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.subsystem.Turret;

@TeleOp(name = "turret test", group = "prototypes")
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

            if (gamepadEx1.isDown(GamepadKeys.Button.LEFT_BUMPER)) turret.setManualPower(gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
            else turret.setManualPower(0.0);

            turret.run();
            turret.printTelemetry();
            Common.telemetry.update();
        }
    }
}
