package org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware;

import static java.lang.Math.abs;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class CachedMotorEx extends MotorEx {

    public double threshold = 0;

    public CachedMotorEx(@NonNull HardwareMap hMap, String id) {
        super(hMap, id);
    }

    public CachedMotorEx(@NonNull HardwareMap hMap, String id, @NonNull GoBILDA gobildaType) {
        super(hMap, id, gobildaType);
    }

    public CachedMotorEx(@NonNull HardwareMap hMap, String id, double cpr, double rpm) {
        super(hMap, id, cpr, rpm);
    }

    public CachedMotorEx reversed() {
        setInverted(true);
        return this;
    }

    private double lastPower = 0;

    public void set(double power) {
        if ((abs(power - lastPower) > threshold) || (power == 0 && lastPower != 0))
            super.set(lastPower = power);
    }
}