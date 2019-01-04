package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

public class SensorColorLCD {

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    final double SCALE_FACTOR = 255;

    View relativeLayout;
    float[] hsvValues = {0F, 0F, 0F};
    final float values[] = hsvValues;
    int relativeLayoutId;

    public void init(HardwareMap ahwMap) {
        sensorColor = ahwMap.get(ColorSensor.class, "sensor_color");
        sensorDistance = ahwMap.get(DistanceSensor.class, "sensor_color");
        relativeLayoutId = ahwMap.appContext.getResources().getIdentifier("RelativeLayout", "id", ahwMap.appContext.getPackageName());
        relativeLayout = ((Activity) ahwMap.appContext).findViewById(relativeLayoutId);
    }

    public float[] getHsvValues() {
        return hsvValues;
    }

    public ColorSensor getSensorColor() {
        return sensorColor;
    }

    public DistanceSensor getSensorDistance() {
        return sensorDistance;
    }

    protected void runSensor() {
        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);


    }
}
