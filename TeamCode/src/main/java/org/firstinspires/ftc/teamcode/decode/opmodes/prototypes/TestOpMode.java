package org.firstinspires.ftc.teamcode.decode.opmodes.prototypes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.subsystem.Feeder;
import org.firstinspires.ftc.teamcode.decode.subsystem.Flywheel;
import org.firstinspires.ftc.teamcode.decode.subsystem.Hood;
import org.firstinspires.ftc.teamcode.decode.subsystem.Intake;

@TeleOp(name = "Test Opmode", group = "prototypes")
public class TestOpMode extends LinearOpMode {
    private Flywheel flywheel;
    private Hood hood;
    private Feeder feeder;
    private Intake intake;

    private GamepadEx gamepadEx1;

    @Override
    public void runOpMode() {
        gamepadEx1 = new GamepadEx(gamepad1);

        feeder = new Feeder(hardwareMap);
        hood = new Hood(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        intake = new Intake(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            gamepadEx1.readButtons();

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) flywheel.set(Flywheel.FlyWheelStates.RUNNING);
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.B)) flywheel.set(Flywheel.FlyWheelStates.ARMING);

            double currentHoodAngle = 0;

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.X)) {
                currentHoodAngle += 30;
                hood.set(currentHoodAngle);
            }

            if (gamepadEx1.wasJustPressed(Y)) {
                currentHoodAngle -= 30;
                hood.set(currentHoodAngle);
            }
            
            if (gamepadEx1.isDown(LEFT_BUMPER)) intake.set(-1.0, true);
            else intake.set(gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), true);

            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.X)) feeder.set(Feeder.FeederStates.RUNNING, false);
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.Y)) feeder.set(Feeder.FeederStates.OUTTAKING, false);
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.B)) feeder.set(Feeder.FeederStates.IDLE, false);
            if (gamepadEx1.wasJustPressed(GamepadKeys.Button.A)) feeder.set(Feeder.FeederStates.OFF, false);

            hood.run();
            intake.run();
            feeder.run();

            hood.printTelemetry();
            flywheel.printTelemetry();
            feeder.printTelemetry();
            Common.telemetry.update();
        }
    }
}
