package org.firstinspires.ftc.teamcode.decode.sensor;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.decode.control.gainmatrices.HSV;
import org.firstinspires.ftc.teamcode.decode.control.gainmatrices.RGB;

public final class ColorSensor {

    private final NormalizedColorSensor sensor;
    private final float[] hsvArray = new float[3];

    private final HSV hsv = new HSV();
    private final RGB rgb = new RGB();

    public ColorSensor(HardwareMap hardwareMap, String name, float gain) {
        sensor = hardwareMap.get(NormalizedColorSensor.class, name);
        sensor.setGain(gain);
        if (sensor instanceof SwitchableLight) ((SwitchableLight) sensor).enableLight(true);
    }

    public void update() {
        NormalizedRGBA rgba = sensor.getNormalizedColors();
        Color.colorToHSV(rgba.toColor(), hsvArray);

        rgb.red = (double) rgba.red * 255;
        rgb.green = (double) rgba.green * 255;
        rgb.blue = (double) rgba.blue * 255;

        hsv.hue = hsvArray[0];
        hsv.saturation = hsvArray[1];
        hsv.value = hsvArray[2];
    }

    public HSV getHSV() {
        return hsv;
    }

    public RGB getRGB() {
        return rgb;
    }
}
