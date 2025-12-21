package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_INTAKE_DISTANCE_SENSOR;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_INTAKE_MOTOR;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.decode.util.DistanceSensorEx;
import org.firstinspires.ftc.teamcode.decode.util.LoopUtil;

@Configurable
public class Intake extends Subsystem<Double> {
    private final MotorEx motor;
    public static float GAIN = 1.0f; //TODO: change gain
    private double power = 0;
    private final DigitalChannel pin0Front, pin0Back;

    public Intake(HardwareMap hardwareMap) {
        this.motor = new MotorEx(hardwareMap, NAME_INTAKE_MOTOR, Motor.GoBILDA.RPM_435);
        pin0Front = hardwareMap.digitalChannel.get(Common.NAME_INTAKE_FRONT_PIN0);
        pin0Back = hardwareMap.digitalChannel.get(Common.NAME_INTAKE_BACK_PIN0);
    }
    public boolean getFrontState() {
        return pin0Front.getState();

    }

    public boolean getBackState() {
        return pin0Back.getState();

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
        Common.telemetry.addData("current power (PERCENTAGE): ", power);
    }
}
