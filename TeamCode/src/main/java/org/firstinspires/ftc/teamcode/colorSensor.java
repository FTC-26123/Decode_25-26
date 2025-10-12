
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@TeleOp(name = "Color Sensor Test"/*, group = "goo" */)
public class colorSensor extends OpMode {
    public NormalizedColorSensor colorSensor;

    boolean detectingPurple;



    @Override
    public void init() {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "revColorV3");
        colorSensor.setGain(7);
        detectingPurple = false;
    }

    public void loop() {
        //return 4 values, red, green, blue, alpha(brightness)
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float normRed, normGreen, normBlue, normPurple;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;
        normPurple = (colors.blue + colors.green) / colors.alpha;


        telemetry.setMsTransmissionInterval(25);

        telemetry.addData("red", normRed);
        telemetry.addData("green", normGreen);
        telemetry.addData("blue", normBlue);
        telemetry.addData("purple", normPurple);

        telemetry.addData("Alpha", colors.alpha);

        telemetry.addLine("Detecting-----------------------------------------------------");

        if (((normGreen + normBlue)/ 1.5) < normRed) {
            telemetry.addLine("Red");
            detectingPurple = false;
        }

        if (((normRed + normBlue)/ 1.3) < normGreen) {
            telemetry.addLine("Green");
            detectingPurple = false;
        }

        if ((((normRed + normGreen) / 1.5) < normBlue) && !detectingPurple) {
            telemetry.addLine("Blue");
            detectingPurple = false;
        }

        if (((((normRed + normBlue) / 1.2 + normGreen))/ 1.5) < normBlue) {
            telemetry.addLine("Purple");
            detectingPurple = true;
        }

        telemetry.update();


    }
}

