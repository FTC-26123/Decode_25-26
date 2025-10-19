/**
 * Please note that Blue is always detected
 * Blue is equal to Purple.
 */
package org.firstinspires.ftc.teamcode.Testing;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name = "Color Sensor Test"/*, group = "goo" */)
public class neel_color extends OpMode {
    public NormalizedColorSensor colorSensor;

    public boolean detectingPurple;
    public boolean detectingBlue;

    //lower- less chance it is detected
    //higher- higher chance it is detected
    //blue and purple should be a little bit more than the others
    //tolerance_blue2 is the tolerance for * purple * # (purple is x5 the other colors)
    public final double tolerance_red = 1.5;
    public final double tolerance_blue = 3.5;
    public final double tolerance_blue2 = 10;
    public final double tolerance_green = 1.3;

    public final double tolerance_purple = 2.3;



    @Override
    public void init() {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "revColorV3");
        colorSensor.setGain(7);
        detectingBlue = false;
        detectingPurple = false;
    }

    public void loop() {
        //return 4 values, red, green, blue, alpha(brightness)
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float normRed, normGreen, normBlue, normPurple;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;
        normPurple = (colors.red + colors.blue) / colors.alpha;


        telemetry.setMsTransmissionInterval(25);

        telemetry.addData("red", normRed);
        telemetry.addData("green", normGreen);
        telemetry.addData("blue", normBlue);
        telemetry.addData("purple", normPurple);

        telemetry.addData("Alpha", colors.alpha);

        telemetry.addLine("Detecting-----------------------------------------------------");

        if (((normGreen + normBlue)/ tolerance_red) < normRed) {
            telemetry.addLine("Red");
            detectingPurple = false;
            detectingBlue = false;
        }


        if (((normRed + normBlue)/ tolerance_green) < normGreen) {
            telemetry.addLine("Green");
            detectingPurple = false;
            detectingBlue = false;
        }

        if (((((normRed + normGreen) / (normPurple * tolerance_blue2) / tolerance_blue) < normBlue)/* && !detectingPurple */))  {
            telemetry.addLine("Blue");
            detectingPurple = false;
            detectingBlue = true;
        }

        if (((((normRed + normBlue) / 1.2 + normGreen)) / tolerance_purple) < normBlue /*&& !detectingBlue*/) {
            telemetry.addLine("Purple");
            detectingPurple = true;
            detectingBlue = false;
        }

        telemetry.update();


    }
}

