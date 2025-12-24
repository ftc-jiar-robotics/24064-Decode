package org.firstinspires.ftc.teamcode.decode.opmodes.auto;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.dashTelemetry;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.robot;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.decode.opmodes.auto.path.ConfigPaths;
import org.firstinspires.ftc.teamcode.decode.opmodes.auto.path.Configuration;
import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.util.Actions;
import org.firstinspires.ftc.teamcode.decode.util.FollowPathAction;

@Autonomous(name = "ConfigAuto")
public class AutoConfig extends AbstractAuto {
    protected Configuration autoConfig;
    ConfigPaths paths;

    @Override
    protected Pose getStartPose() {
        return Configuration.isBigTriangle() ? ConfigPaths.startClose : ConfigPaths.startFar;
    }

    @Override
    protected void configure() {
        paths = new ConfigPaths(robot.drivetrain);
        autoConfig = new Configuration(paths);

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        Common.dashTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        int screenNumber = 0;

        boolean isFinished = false;
        boolean isConfirmed = false;

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
            } else if (!isConfirmed) {
                dashTelemetry.addLine("SHOW TO DRIVER COACHES");
                dashTelemetry.addLine("Path Combination: " + String.join(" ", autoConfig.getRequirementList().toString()));

                if (gamepadEx1.wasJustPressed(DPAD_LEFT)) isFinished = false;
                if (gamepadEx1.wasJustPressed(A)) isConfirmed = true;
            }

            if (isConfirmed) {
                dashTelemetry.clear();
                dashTelemetry.addLine("CONFIRMED!");
                dashTelemetry.addLine("Path Combination: " + String.join(" ", autoConfig.getRequirementList().toString()));
                try {
                    autoConfig.getRequirementList().get(0).getAction().call().run(new TelemetryPacket());
                    autoConfig.getRequirementList().get(1).getAction().call().run(new TelemetryPacket());

                    robot.actionScheduler.runBlocking();
                } catch (Exception e) {
                    throw new RuntimeException(e);
                }

                if (Common.isRed != ConfigPaths.isPathRed) {
                    ConfigPaths.isPathRed = !ConfigPaths.isPathRed;
                    paths.mirrorAll();
                }
                Common.robot.shooter.setGoalAlliance();
                paths.configAutoBuild(Configuration.isBigTriangle());
            }
        }

    }

    @Override
    protected void onRun() {
        for (Configuration.Option o : autoConfig.getRequirementList()) {
            try {
                Log.d("AutoConfig", o.name());
                robot.actionScheduler.addAction(new Actions.UntilConditionAction(() -> getRuntime() > ConfigPaths.LEAVE_TIME, o.getAction().call()));
                robot.actionScheduler.runBlocking();
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }
        robot.actionScheduler.addAction(new FollowPathAction(robot.drivetrain, paths.leave, true));
        robot.actionScheduler.runBlocking();
    }
}
