package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestBenchColor {
    NormalizedColorSensor colorSensor;

    public enum DetectedColor {
        BLUE,
        RED,
        GREEN,
        UNKNOWN
    }
    public void init(HardwareMap hwMap){
        colorSensor=hwMap.get(NormalizedColorSensor.class,"revColorV3");
//      Below: Calibrate Sensor
        colorSensor.setGain(7);
    }

    public DetectedColor getDetectedColor(Telemetry telemetry){
        NormalizedRGBA colors = colorSensor.getNormalizedColors(); //Returns values

        float normBlue, normRed, normGreen, normPurple;
        normBlue = colors.blue / colors.alpha;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normPurple = (colors.red + colors.blue) / colors.alpha;

        telemetry.addData("blue:", normBlue);
        telemetry.addData("red:", normRed);
        telemetry.addData("green:", normGreen);
        telemetry.addData("purple", normPurple);

        telemetry.setMsTransmissionInterval(30);

        if (((normBlue+normGreen)/2)<normRed){
            telemetry.addLine("The color is: RED");
        }
        if (((normBlue+normRed)/1.5)<normGreen){
            telemetry.addLine("The color is: GREEN");
        }
        if (((normRed+normGreen)/1.1)<normBlue){
            telemetry.addLine("The color is: BLUE");
        }

        if((normGreen+normRed+normBlue)/0.9<normPurple){
            telemetry.addLine("The Color Is: PURPLE");
        }

        telemetry.update();

        return DetectedColor.UNKNOWN;
    }
}
