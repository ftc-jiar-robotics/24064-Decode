package org.firstinspires.ftc.teamcode.decode.opmodes.prototypes;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_TURRET_MOTOR;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_TURRET_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name = "TurretEncoderDebug", group = "Debug")
public class TurretEncoderDebug extends LinearOpMode {
    @Override public void runOpMode() {
        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, NAME_TURRET_MOTOR);
        AnalogInput abs  = hardwareMap.get(AnalogInput.class, NAME_TURRET_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("turret ticks", turret.getCurrentPosition());
            telemetry.addData("abs angle (raw deg)", abs.getVoltage() / 3.2 * 360.0);
            telemetry.update();
        }
    }
}
