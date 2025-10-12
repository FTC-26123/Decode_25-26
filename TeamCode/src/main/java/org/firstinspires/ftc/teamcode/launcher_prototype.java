
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@TeleOp(name = "Launcher Prototype"/*, group = "goo" */)
public class launcher_prototype extends OpMode {
    public DcMotor launchMotor;
    public DcMotor intakeMotor;
    public DcMotor windmill;

    public double motor_power = 1;

    String motor_direction = "FORWARD";



    @Override
    public void init() {

        launchMotor = hardwareMap.get(DcMotor.class, "rightBack"); // Motors are named after Roadrunner Config- Panel 0
        intakeMotor = hardwareMap.get(DcMotor.class, "rightFront"); // Panel 1
        windmill = hardwareMap.get(DcMotor.class, "windmill"); // Panel 0

        launchMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        windmill.setDirection(DcMotor.Direction.FORWARD);

    }

    @Override
    public void loop() {

        telemetry.setMsTransmissionInterval(20);

        telemetry.addLine("Rotate Launch Motor = gamepad1.a");
        telemetry.addLine("Rotate Intake Motor = gamepad1.b");
        telemetry.addLine("Rotate Windmill = gamepad1.x");
        telemetry.addLine("Set Direction Forward = gamepad1.left_bumper");
        telemetry.addLine("Set Direction Reverse = gamepad1.right_bumper");
        telemetry.addLine("Add Power + 0.1 = gamepad1.dpad_left");
        telemetry.addLine("Add Power - 0.1 = gamepad1.dpad_right");
        telemetry.addLine("Turn All Motors Off = gamepad1.ps");

        telemetry.addData("Motor Power", motor_power);
        telemetry.addData("Direction", motor_direction);
        telemetry.addData("Transmit Interval (ms)", telemetry.getMsTransmissionInterval());

        telemetry.addData("Launch", launchMotor.getPower());
        telemetry.addData("Intake", intakeMotor.getPower());
        telemetry.addData("Windmill", windmill.getPower());

        // forward - lb
        if (gamepad1.left_bumper) {

            motor_direction = "FORWARD";
            launchMotor.setDirection(DcMotor.Direction.FORWARD);
            intakeMotor.setDirection(DcMotor.Direction.FORWARD);
            windmill.setDirection(DcMotor.Direction.FORWARD);
        }

        // backward- rb
        if (gamepad1.right_bumper) {

            motor_direction = "REVERSE";
            launchMotor.setDirection(DcMotor.Direction.REVERSE);
            intakeMotor.setDirection(DcMotor.Direction.REVERSE);
            windmill.setDirection(DcMotor.Direction.REVERSE);
        }

        // add 0.1 to m_p
        if (gamepad1.dpad_left & (motor_power <= 1) & (motor_power >= 0.1)) {
            motor_power = motor_power + 0.1;
        }

        // subtract 0.1 to m_p
        if (gamepad1.dpad_right & (motor_power <= 1) & (motor_power >= 0.1)) {
            motor_power = motor_power - 0.1;
        }

        //off- ps
        if (gamepad1.psWasPressed())
            motor_power = 0;

        // launch
        if (gamepad1.a) {
            launchMotor.setPower(motor_power);
        }


        // intake
        if (gamepad1.b) {
            intakeMotor.setPower(motor_power);
        }

        // windmill
        if (gamepad1.x) {
            windmill.setPower(motor_power);
        }

        telemetry.update();

    }
}
