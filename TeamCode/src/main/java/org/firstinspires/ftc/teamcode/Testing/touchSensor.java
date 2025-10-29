package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Touch Sensor Test")
public class touchSensor extends OpMode {


    public DigitalChannel touch;

    public CRServo gate;

    public Servo light1;


    @Override
    public void init() {

        touch=hardwareMap.get(DigitalChannel.class, "touch");
        touch.setMode(DigitalChannel.Mode.INPUT);

        gate=hardwareMap.get(CRServo.class, "gate");

        light1=hardwareMap.get(Servo.class, "light1");

    }

    public boolean touchSensorState() {
        return touch.getState();
    }

//    @Override
//    public void start() {
//
//    }

    @Override
    public void loop() {

        telemetry.addLine("Blue means touch, Red means no touch");

        telemetry.addData("Touch Sensor State", touchSensorState());

        if (gamepad2.y) {
            gate.setPower(0.2);
        }
        if (gamepad2.b) {
            gate.setPower(-0.2);
        }

//        if (gamepad2.ps) {
//            gate.setPower(0);
//        }

        if (!gamepad2.b && !gamepad2.y && !gamepad2.x) {
            gate.setPower(0);
        }

        //state is backwards (touching is false, not touching is true)
        if (!touchSensorState()) {
            light1.setPosition(0.611);
            gate.setPower(0);
        } else if (touchSensorState()) {
            light1.setPosition(0.29);
        }




    }
}
