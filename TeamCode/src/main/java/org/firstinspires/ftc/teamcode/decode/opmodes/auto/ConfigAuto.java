package org.firstinspires.ftc.teamcode.decode.opmodes.auto;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_STICK_BUTTON;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.dashTelemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.isSlowMode;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.decode.subsystem.AutoConfig;
import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
@Autonomous(name = "ConfigAuto")
public class ConfigAuto extends AbstractAuto {
    protected AutoConfig autoConfig;

    @Override
    protected Pose getStartPose() {
        return null;
    }

    @Override
    protected void configure() {
        autoConfig = new AutoConfig();

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        Common.dashTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        int screenNumber = 0;

        boolean isFinished = false;

        while (opModeInInit()) {
            gamepadEx1.readButtons();
            dashTelemetry.update();

            if (!isFinished) {
                isFinished = gamepadEx1.isDown(LEFT_BUMPER) && gamepadEx1.isDown(RIGHT_BUMPER);

                boolean isCursorDown = gamepadEx1.wasJustPressed(DPAD_UP);
                boolean isCursorUp = gamepadEx1.wasJustPressed(DPAD_DOWN);
                dashTelemetry.clear();
                switch (screenNumber) {
                    case 0:
                        autoConfig.doConfig(autoConfig.allianceChoices, isCursorUp, isCursorDown, screenNumber);
                        break;
                    case 1:
                        autoConfig.doConfig(autoConfig.sideChoices, isCursorUp, isCursorDown, screenNumber);
                        break;
                    default:
                        autoConfig.doConfig(autoConfig.pathChoices, isCursorUp, isCursorDown, screenNumber);
                        break;
                }

                if (gamepadEx1.wasJustPressed(DPAD_RIGHT)) {
                    screenNumber++;
                    dashTelemetry.clear();
                }

                if (gamepadEx1.wasJustPressed(DPAD_LEFT)) {
                    if (screenNumber > 0) screenNumber--;
                    dashTelemetry.clear();
                }
                autoConfig.reduceRequireList(screenNumber);
            } else {
                dashTelemetry.addLine("SHOW TO DRIVER COACHES");
                dashTelemetry.addLine("Path Combination: " + String.join(" ", autoConfig.getRequirementList().toString()));

                if (gamepadEx1.wasJustPressed(DPAD_LEFT)) isFinished = false;
            }
        }

    }

    @Override
    protected void onRun() {

    }
}
