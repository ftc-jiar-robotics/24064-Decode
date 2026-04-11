package org.firstinspires.ftc.teamcode.decode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Config
public class CachedCRServo {

    private final CRServo servo;

    // Last power actually sent to the hardware
    private double currentOutput = 0.0;

    private double roundingPoint;

    public CachedCRServo(@NonNull HardwareMap hardwareMap, @NonNull String id, double roundingPoint) {
        this.roundingPoint = roundingPoint;
        this.servo = hardwareMap.get(CRServo.class, id);
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        servo.setDirection(direction);
    }

    public void setRoundingPoint(double roundingPoint) {
        this.roundingPoint = roundingPoint;
    }

    public double getCurrentOutput() {
        return currentOutput;
    }

    public CRServo getRawServo() {
        return servo;
    }

    public void setPower(double power) {
        // Clamp incoming command
        double output = Range.clip(power, -1.0, 1.0);

        if (roundingPoint > 0) {
            if (output > 0) {
                output = Math.floor(output * roundingPoint) / roundingPoint;
            } else if (output < 0) {
                output = Math.ceil(output * roundingPoint) / roundingPoint;
            } else {
                output = 0;
            }
        }

        if (currentOutput != output || (currentOutput != 0 && output == 0)) {
            servo.setPower(output);
            currentOutput = output;
        }
    }
}
