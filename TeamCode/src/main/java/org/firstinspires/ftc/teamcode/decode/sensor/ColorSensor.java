package org.firstinspires.ftc.teamcode.decode.sensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.HSV;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrix.RGB;

public final class ColorSensor {

    private final NormalizedColorSensor sensor;

    public HSV hsv = new HSV();
    public RGB rgb = new RGB();

    public ColorSensor(HardwareMap hardwareMap, String name, float gain) {
        sensor = hardwareMap.get(NormalizedColorSensor.class, name);
        sensor.setGain(gain);
        setLightOn(true);
    }

    public void setLightOn(boolean lightOn) {
        if (sensor instanceof SwitchableLight) ((SwitchableLight) sensor).enableLight(lightOn);
    }

    public void update() {
        NormalizedRGBA rgba = sensor.getNormalizedColors();

        rgb = new RGB(
                rgba.red * 255 * rgba.alpha,
                rgba.green * 255 * rgba.alpha,
                rgba.blue * 255 * rgba.alpha
        );

        hsv = rgb.toHSV();
    }
}