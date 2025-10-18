package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Drive_Test")
public class drivetest extends OpMode {

    public DcMotor motor1;
    public DcMotor motor2;
    public DcMotor motor3;
    public DcMotor motor4;
    private CRServo test;



    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        test = hardwareMap.get(CRServo.class, "test");
    }

    @Override
    public void loop() {


//        Forward/Backward Movement
        if (gamepad1.a) {
           motor1.setPower(1);
        }
        if(gamepad1.b){
            motor2.setPower(1);
        }
        if(gamepad1.y){
            motor3.setPower(1);
        }
        if(gamepad1.x){
            motor4.setPower(1);
        }
        if(gamepad1.dpad_up){
            motor1.setPower(1);
            motor2.setPower(1);
            motor3.setPower(1);
            motor4.setPower(1);
        }
        if(gamepad1.ps){
            motor1.setPower(0);
            motor2.setPower(0);
            motor3.setPower(0);
            motor4.setPower(0);
            test.setPower(0);
        }
        if(gamepad1.dpad_down){
            test.setPower(1);
        }
    }
}