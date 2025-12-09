package org.firstinspires.ftc.teamcode.decode.opmodes.prototypes;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_TURRET_MOTOR;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_TURRET_ENCODER;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.decode.util.BulkReader;

@TeleOp(name = "TurretEncoderDebug", group = "Debug")
public class TurretEncoderDebug extends LinearOpMode {
    @Override public void runOpMode() {
        Motor.Encoder motor = new MotorEx(hardwareMap, "right back", Motor.GoBILDA.RPM_1150).encoder;
        AnalogInput abs  = hardwareMap.get(AnalogInput.class, NAME_TURRET_ENCODER);
        BulkReader bulkReader = new BulkReader(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            bulkReader.bulkRead();
            telemetry.addData("turret motor enc (ticks)", motor.getPosition());
            telemetry.addData("abs angle (raw deg)", abs.getVoltage() / 3.2 * 360.0);
            telemetry.update();
        }
    }
}
