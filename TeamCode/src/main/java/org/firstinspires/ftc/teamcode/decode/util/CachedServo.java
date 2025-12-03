package org.firstinspires.ftc.teamcode.decode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class CachedServo extends SimpleServo {
    private double currentOutput;

    public static double roundingPoint = 1000;

    public CachedServo(@NonNull HardwareMap hardwareMap, String id, double minAngle, double maxAngle, AngleUnit angleUnit) {
        super(hardwareMap, id, minAngle, maxAngle, angleUnit);
    }

    @Override
    public void turnToAngle(double output) {
        if (output > 0) {
            output = Math.floor(output * roundingPoint) / roundingPoint;
        } else if (output < 0) {
            output = Math.ceil(output * roundingPoint) / roundingPoint;
        } else {
            output = 0;
        }

        if (currentOutput != output || (currentOutput != 0 && output == 0)) {
            super.turnToAngle(output);
            currentOutput = output;
        }
    }
}
