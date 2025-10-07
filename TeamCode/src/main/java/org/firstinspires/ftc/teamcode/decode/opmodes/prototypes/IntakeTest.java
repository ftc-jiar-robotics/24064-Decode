package org.firstinspires.ftc.teamcode.decode.opmodes.prototypes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.subsystem.Intake;
@TeleOp(name = "intake test", group = "prototypes")
public class IntakeTest extends LinearOpMode {
    private GamepadEx gamepadEx1;
    private Intake intake;
    @Override
    public void runOpMode() {
        gamepadEx1 = new GamepadEx(gamepad1);

        intake = new Intake(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            gamepadEx1.readButtons();
            intake.set(gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), true);
            intake.run();
        }
    }
}
