package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_INTAKE_COLORSENSOR;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_INTAKE_MOTOR;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.decode.sensor.ColorSensor;

@Configurable
public class Intake extends Subsystem<Double> {
    private final MotorEx motor;
    private final ColorSensor colorSensor;
    private float gain = 0; //TODO: change gain
    private double power = 0;
    public Intake(HardwareMap hardwareMap) {
        this.motor = new MotorEx(hardwareMap, NAME_INTAKE_MOTOR);
        this.colorSensor = new ColorSensor(hardwareMap, NAME_INTAKE_COLORSENSOR, gain);
    }

    @Override
    public void set(Double power) {
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

    @Override
    public void printTelemetry() {
        Common.telemetry.addLine("INTAKE");
        Common.telemetry.addData("current power: ", power);
    }
}
