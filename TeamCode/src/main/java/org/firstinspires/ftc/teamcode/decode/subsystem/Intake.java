package org.firstinspires.ftc.teamcode.decode.subsystem;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.decode.sensor.ColorSensor;

public class Intake extends Subsystem<Double> {
    private final MotorEx motor;
    private final ColorSensor colorSensor;
    private float gain = 0; //TODO: change gain
    private double power = 0;
    public Intake(HardwareMap hardwareMap) {
        this.motor = hardwareMap.get(MotorEx.class,"intake");
        this.colorSensor = new ColorSensor(hardwareMap,"color",gain);
    }

    @Override
    protected void set(Double power) {
        this.power = power;
    }

    @Override
    public Double get() {
        return power;
    }

    @Override
    public void run() {
        motor.set(power);
    }
}
