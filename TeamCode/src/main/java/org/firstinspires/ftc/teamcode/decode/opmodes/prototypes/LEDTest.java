package org.firstinspires.ftc.teamcode.decode.opmodes.prototypes;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.subsystem.LEDController;

@TeleOp(name = "LED test", group = "prototypes")
public class LEDTest extends LinearOpMode {
    private LEDController ledController;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        ledController = new LEDController(hardwareMap);

        RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_FOREST_PALETTE;
        while (opModeIsActive()) {
            pattern = pattern.next();
            ledController.setPattern(pattern);
            telemetry.addData("Pattern", pattern);
            telemetry.update();
            sleep(1000);

        }
    }
}
