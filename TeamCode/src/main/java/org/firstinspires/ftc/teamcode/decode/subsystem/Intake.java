package org.firstinspires.ftc.teamcode.decode.subsystem;

import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_INTAKE_COLOR_SENSOR;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.NAME_INTAKE_MOTOR;
import static org.firstinspires.ftc.teamcode.decode.subsystem.Common.telemetry;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.decode.sensor.ColorSensor;
import org.firstinspires.ftc.teamcode.decode.util.CachedMotor;
import org.firstinspires.ftc.teamcode.decode.util.LoopUtil;

@Configurable
public class Intake extends Subsystem<Double> {
    private final CachedMotor motor;
    private final ColorSensor colorSensor;
    public static float GAIN = 1.0f; //TODO: change gain
    private double power = 0;
    public Intake(HardwareMap hardwareMap) {
        this.motor = new CachedMotor(hardwareMap, NAME_INTAKE_MOTOR, Motor.GoBILDA.RPM_435);
        this.colorSensor = new ColorSensor(hardwareMap, NAME_INTAKE_COLOR_SENSOR, GAIN);
    }

    @Override
    public void set(Double power) {
        this.power = power;
    }

    public Robot.ArtifactColor getColor() {
        return Robot.getColor(colorSensor, false);
    }

    @Override
    public Double get() {
        return power;
    }

    @Override
    public void run() {
        motor.set(power);

        if ((LoopUtil.getLoops() & Common.COLOR_SENSOR_UPDATE_LOOPS) == 0)
            colorSensor.update();
    }

    @Override
    public void printTelemetry() {
        Common.telemetry.addLine("INTAKE");
        Common.telemetry.addData("current power (PERCENTAGE): ", power);

        Common.telemetry.addLine("COLOR SENSOR");
        telemetry.addData("curr color (ENUM): ", getColor());
        telemetry.addData("curr color (HSV): ", colorSensor.hsv);
    }
}
