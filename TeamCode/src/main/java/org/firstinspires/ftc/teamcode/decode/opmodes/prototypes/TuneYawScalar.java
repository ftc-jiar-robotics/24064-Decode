package org.firstinspires.ftc.teamcode.decode.opmodes.prototypes;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.decode.util.LoopUtil;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(group = "prototypes")
public class TuneYawScalar extends LinearOpMode {
    public static double YAW_SCALAR = 1.00086;

    @Override
    public void runOpMode() throws InterruptedException {
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        Follower drivetrain = Constants.createFollower(hardwareMap);

        waitForStart();
        pinpoint.resetPosAndIMU();
        drivetrain.startTeleOpDrive();

        while(opModeIsActive()) {
            drivetrain.update();
            drivetrain.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            pinpoint.setYawScalar(YAW_SCALAR);
            pinpoint.update();
            telemetry.addData("Heading (deg)", pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Loop time (hertz)", LoopUtil.getLoopTimeInHertz());
            telemetry.update();
        }
    }
}
