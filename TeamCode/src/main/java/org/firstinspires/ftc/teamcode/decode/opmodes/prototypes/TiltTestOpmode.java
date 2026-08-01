package org.firstinspires.ftc.teamcode.decode.opmodes.prototypes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.decode.subsystem.ParkLift;

@TeleOp(name = "tilt test", group = "prototypes")
public class TiltTestOpmode extends LinearOpMode {
    private GamepadEx gamepadEx1;
    private ParkLift tilt;
    @Override
    public void runOpMode() {
        gamepadEx1 = new GamepadEx(gamepad1);

        tilt = new ParkLift(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            gamepadEx1.readButtons();
            if (gamepadEx1.isDown(DPAD_RIGHT)) tilt.politeSet(ParkLift.ParkStates.DOWN);
            if (gamepadEx1.isDown(DPAD_LEFT)) tilt.politeSet(ParkLift.ParkStates.UP);
            if (gamepadEx1.isDown(DPAD_UP)) tilt.politeSet(ParkLift.ParkStates.RESET);

            tilt.run();
        }
    }
}
