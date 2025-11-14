package org.firstinspires.ftc.teamcode.decode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class CachedMotor extends MotorEx {
    private double currentOutput;
    
    public static double
            ROUNDING_POINT = 1000,
            SLEW_RATE = 0.2;

    public CachedMotor(@NonNull HardwareMap hardwareMap, String id, @NonNull GoBILDA gobildaType) {
        super(hardwareMap, id, gobildaType);
    }

    @Override
    public void set(double output) {
        if (output > 0) {
            output = Math.floor(output * ROUNDING_POINT) / ROUNDING_POINT;
        } else if (output < 0) {
            output = Math.ceil(output * ROUNDING_POINT) / ROUNDING_POINT;
        } else {
            output = 0;
        }

        if (currentOutput != output || (currentOutput != 0 && output == 0)) {
            SLEW_RATE = 0.2;
            double desiredChange = output - currentOutput;
            double limitedChange = Math.max(-SLEW_RATE, Math.min(desiredChange, SLEW_RATE));
            super.set(currentOutput += limitedChange);

            currentOutput = output;
        }
    }
}
