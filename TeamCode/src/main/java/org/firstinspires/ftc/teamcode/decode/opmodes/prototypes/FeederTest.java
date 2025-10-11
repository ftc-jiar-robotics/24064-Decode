package org.firstinspires.ftc.teamcode.decode.opmodes.prototypes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.subsystem.Feeder;

@TeleOp(name = "feeder test", group = "prototypes")
public class FeederTest extends LinearOpMode {
    private GamepadEx gamepadEx1;
    private Feeder feeder;

    @Override
    public void runOpMode() {
        gamepadEx1 = new GamepadEx(gamepad1);

        feeder = new Feeder(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            gamepadEx1.readButtons();

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.X)) {
                feeder.set(Feeder.FeederStates.RUNNING, false);
            }
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.Y)) {
                feeder.set(Feeder.FeederStates.IDLE, false);
            }
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.B)) {
                feeder.set(Feeder.FeederStates.OUTTAKING, false);
            }
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) {
                feeder.set(Feeder.FeederStates.OFF, false);
            }

            feeder.run();

            feeder.printTelemetry();
            Common.telemetry.update();
        }
    }
}
