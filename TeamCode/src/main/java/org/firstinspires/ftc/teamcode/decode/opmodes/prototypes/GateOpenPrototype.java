package org.firstinspires.ftc.teamcode.decode.opmodes.prototypes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.subsystem.Intake;

import org.firstinspires.ftc.teamcode.decode.subsystem.GateOpener;
@TeleOp(name = "Gate Open Prototype", group = "prototypes")
public class GateOpenPrototype extends LinearOpMode {
    private GamepadEx gamepadEx1;
    private GateOpener gateOpener;
    @Override
    public void runOpMode() {
        gamepadEx1 = new GamepadEx(gamepad1);

        gateOpener = new GateOpener(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            gamepadEx1.readButtons();
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) {
//                gateOpener.set(!gateOpener.get(), true);
            }
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.B)) {
                Common.isRed = !Common.isRed;
            }

            gateOpener.run();
        }
    }
}
