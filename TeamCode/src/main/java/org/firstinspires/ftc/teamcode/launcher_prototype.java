package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
@TeleOp(name = "Launcher Prototype")
public class launcher_prototype extends OpMode {

    public DcMotor launcher;
    public DcMotor intake;
    public DcMotor windmill;
    public Servo sorter;

    public NormalizedColorSensor colorSensor;

    @Override
    public void init() {

        launcher = hardwareMap.get(DcMotor.class, "launcher"); // Motors are named after Roadrunner Config- Panel 0
        intake = hardwareMap.get(DcMotor.class, "intake"); // Panel 1
        windmill = hardwareMap.get(DcMotor.class, "windmill"); // Panel// 0
        sorter = hardwareMap.get(Servo.class, "gate");

        launcher.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        windmill.setDirection(DcMotor.Direction.FORWARD);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "revColorV3");
        colorSensor.setGain(7);

    }

    @Override
    public void loop() {

        telemetry.setMsTransmissionInterval(20);

        telemetry.addLine("Rotate Launch Motor = gamepad1.a");
        telemetry.addLine("Rotate Intake Motor = gamepad1.b");
        telemetry.addLine("Rotate Windmill = gamepad1.x");
        telemetry.addLine("Neutral = gamepad1.y");
        telemetry.addLine("Pos1 = gamepad1.right_stick_right");
        telemetry.addLine("Pos2 = gamepad1.right_stick_left");
        telemetry.addLine("Turn All Motors Off = gamepad1.ps");
        telemetry.addLine("Rotate Intake Power 0.4- gamepad1.dpad_left");
        telemetry.addLine("Launch power 0.5- gamepad1.dpad_up, " +
                "0.7- gamepad1.dpad_right, ");
        telemetry.addLine("Launch power 0.9- gamepad1.dpad_down");

        telemetry.addData("Transmit Interval (ms)", telemetry.getMsTransmissionInterval());

        telemetry.addData("Launch Power", launcher.getPower());
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.addData("Windmill Power", windmill.getPower());
        telemetry.addData("Sorter Position", sorter.getPosition());


        //off- ps
        if (gamepad1.ps) {
            launcher.setPower(0);
            intake.setPower(0);
            windmill.setPower(0);
        }

        // launch
        if (gamepad1.a) {
            launcher.setPower(1);
        }

        if (gamepad1.dpad_up) {
            launcher.setPower(0.5);
        }

        else if (gamepad1.dpad_right) {
            launcher.setPower(0.7);
        }

        else if (gamepad1.dpad_down) {
            launcher.setPower(0.9);
        }



        // intake
        if (gamepad1.b) {
            intake.setPower(1);
        }

        if (gamepad1.dpad_left) {
            intake.setPower(0.4);
        }

        // windmill
        if (gamepad1.x) {
            windmill.setPower(1);
        }

        if (gamepad1.y) {
            sorter.setPosition(0.5);
        }

        if (gamepad1.right_stick_x > 0) {
            sorter.setPosition(1);
        }

        if (gamepad1.right_stick_x < 0) {
            sorter.setPosition(0);
        }

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


        telemetry.update();

    }
}


