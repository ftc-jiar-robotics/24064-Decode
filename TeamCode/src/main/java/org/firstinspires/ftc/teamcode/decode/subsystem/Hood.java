package org.firstinspires.ftc.teamcode.decode.subsystem;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Hood extends Subsystem<Double> {
    private final SimpleServo hood;

    private double targetAngle;

    public Hood(HardwareMap hw) {
        this.hood = new SimpleServo(hw, "hood", Common.SERVO_AXON_MIN, Common.SERVO_AXON_MAX_1);
    }

    public Double get() {
        return targetAngle;
    }

    protected void set(Double a) {
        targetAngle = a;
    }

    public void run() {
        hood.turnToAngle(targetAngle);
    }
}
