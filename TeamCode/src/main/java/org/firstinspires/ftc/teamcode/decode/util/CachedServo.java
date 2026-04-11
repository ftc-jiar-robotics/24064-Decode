package org.firstinspires.ftc.teamcode.decode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class CachedServo extends SimpleServo {
    private double currentOutput;

    public double roundingPoint = 1000;

    private final ServoImplEx servoImplEx;

    public CachedServo(@NonNull HardwareMap hardwareMap, String id, double minAngle, double maxAngle, AngleUnit angleUnit) {
        this(hardwareMap, id, minAngle, maxAngle, angleUnit, 100);
    }
    public CachedServo(@NonNull HardwareMap hardwareMap, String id, double minAngle, double maxAngle, AngleUnit angleUnit, double roundingPoint) {
        super(hardwareMap, id, minAngle, maxAngle, angleUnit);
        this.roundingPoint = roundingPoint;
        servoImplEx = hardwareMap.get(ServoImplEx.class, id);
    }

    public void setPwmRange(double minUs, double maxUs) {
        servoImplEx.setPwmRange(new PwmControl.PwmRange(minUs, maxUs));
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
