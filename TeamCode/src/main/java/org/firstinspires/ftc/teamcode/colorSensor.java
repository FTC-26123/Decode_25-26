package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
@TeleOp(name = "Color Sensor Test" , group = "Color")
public class colorSensor extends OpMode {
    public NormalizedColorSensor colorSensor;

    @Override
    public void init() {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "revColorV3");
        colorSensor.setGain(7);


    }

    public void loop() {

        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        int col = colors.toColor();
        double hue = JavaUtil.colorToHue(col);

        telemetry.addData("Detected Hue", hue);
        telemetry.addData("Detected Color", col);

        if (hue > 15 && hue < 60) {
            telemetry.addLine("Detected Color: Red");
        } else if (hue > 90 && hue < 180) {
            telemetry.addLine("Detected Color: Green");
        } else if (hue > 200 && hue < 210) {
            telemetry.addLine("Detected Color: Blue");
        } else if (hue > 225 && hue < 350) {
            telemetry.addLine("Detected Color: Purple");
        }

        telemetry.setMsTransmissionInterval(25);

        telemetry.update();


    }
}

