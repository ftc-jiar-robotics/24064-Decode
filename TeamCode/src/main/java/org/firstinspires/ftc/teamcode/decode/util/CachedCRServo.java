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

    // How finely we quantize power. Example:
    // 100.0 => steps of 0.01, 20.0 => steps of 0.05, 0.0 => no quantization
    public static double POWER_ROUNDING_POINT = 100.0;

    public static double SLEW_RATE = 0.2;

    public CachedCRServo(@NonNull HardwareMap hardwareMap, @NonNull String id) {
        this.servo = hardwareMap.get(CRServo.class, id);
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        servo.setDirection(direction);
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

        if (POWER_ROUNDING_POINT > 0) {
            if (output > 0) {
                output = Math.floor(output * POWER_ROUNDING_POINT) / POWER_ROUNDING_POINT;
            } else if (output < 0) {
                output = Math.ceil(output * POWER_ROUNDING_POINT) / POWER_ROUNDING_POINT;
            } else {
                output = 0;
            }
        }

        // If nothing changed and we are not going to 0, do nothing
        if (output == currentOutput && !(currentOutput != 0 && output == 0)) {
            return;
        }

        double desiredChange = output - currentOutput;
        double limitedChange = Range.clip(desiredChange, -SLEW_RATE, SLEW_RATE);
        double newOutput = currentOutput + limitedChange;

        servo.setPower(newOutput);
        currentOutput = newOutput;
    }
}
