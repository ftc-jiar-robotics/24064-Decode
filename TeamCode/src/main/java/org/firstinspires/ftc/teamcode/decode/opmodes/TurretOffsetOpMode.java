package org.firstinspires.ftc.teamcode.decode.opmodes;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.dashTelemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.decode.subsystem.Common;
import org.firstinspires.ftc.teamcode.decode.subsystem.Turret;

@TeleOp(name = "turret offset opmode", group = "24064")
public class TurretOffsetOpMode extends LinearOpMode {
    public Turret turret;

    @Override
    public void runOpMode() throws InterruptedException {
        turret = new Turret(hardwareMap);

        waitForStart();

        double currentAngle = turret.getAbsoluteEncoderAngle();
        Common.dashTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (opModeIsActive()) {
            dashTelemetry.addData("OFFSET (DEGREES): ", Turret.ABSOLUTE_ENCODER_OFFSET + currentAngle);
            dashTelemetry.update();
        }
    }
}
