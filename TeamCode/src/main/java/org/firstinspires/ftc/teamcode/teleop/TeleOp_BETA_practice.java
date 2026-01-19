package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TeleOp_Beta")
public class TeleOp_BETA_practice extends OpMode {

    public final float MOTOR_MULTIPLIER_PERCENTAGE_CAP = 0.55F;
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    public DcMotorEx launcherRight;
    public DcMotor index;
    public DcMotor intake;
    public DcMotorEx launcherLeft;

    public float frontLeftMotorSpeed = 0;
    public float frontRightMotorSpeed = 0;
    public float backLeftMotorSpeed = 0;
    public float backRightMotorSpeed = 0;
    long dualVelocity = 2250;

    public void update() {
//     Robot Motor Power Limits
        frontLeftMotor.setPower(frontLeftMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        frontRightMotor.setPower(frontRightMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        backLeftMotor.setPower(backLeftMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        backRightMotor.setPower(backRightMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);
    }

    @Override
    public void init() {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        launcherRight = hardwareMap.get(DcMotorEx.class, "launcher");
        index = hardwareMap.get(DcMotor.class, "windmill");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcherLeft = hardwareMap.get(DcMotorEx.class, "launcherLeft");

        launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);;
        index.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.setMsTransmissionInterval(5);


    }

    @Override
    public void loop() {

        frontLeftMotorSpeed = 0;
        frontRightMotorSpeed = 0;
        backLeftMotorSpeed = 0;
        backRightMotorSpeed = 0;

        float left_stick_x = gamepad1.left_stick_x;
        float left_stick_y = gamepad1.left_stick_y;
        float right_stick_x = gamepad1.right_stick_x;

//        Forward/Backward Movement
        if (left_stick_y != 0) {
            frontLeftMotorSpeed = -left_stick_y;
            frontRightMotorSpeed = -left_stick_y;
            backLeftMotorSpeed = -left_stick_y;
            backRightMotorSpeed = -left_stick_y;
        }

//        Lateral Movement
        if (left_stick_x != 0) {
            frontLeftMotorSpeed += left_stick_x;
            frontRightMotorSpeed -= left_stick_x;
            backLeftMotorSpeed -= left_stick_x;
            backRightMotorSpeed += left_stick_x;
        }

//        Rotation
        if (right_stick_x != 0) {
            frontLeftMotorSpeed += right_stick_x;
            backLeftMotorSpeed += right_stick_x;
            frontRightMotorSpeed -= right_stick_x;
            backRightMotorSpeed -= right_stick_x;
        }

        if (gamepad2.a) {
            intake.setPower(1);
            index.setPower(1);
        } else if (gamepad2.b) {
            intake.setPower(-1);
            index.setPower(-1);
        } else if (!gamepad2.a && !gamepad2.b) {
            intake.setPower(0);
            index.setPower(0);
        }

        telemetry.addData("Launcher Left Velocity", launcherLeft.getVelocity());
        telemetry.addData("Launcher Right Velocity", launcherRight.getVelocity());
        telemetry.addData("Target Launcher Velocity", dualVelocity);
        if (gamepad2.left_trigger > 0.4) {
            launcherLeft.setVelocity(dualVelocity);
            launcherRight.setVelocity(dualVelocity);
        } if (gamepad2.left_bumper) {
            launcherLeft.setVelocity(0);
            launcherRight.setVelocity(0);
        }

        telemetry.addLine("Gamepad2.dpad_up -> increase velocity by 10");
        if (gamepad2.dpad_up) {
            dualVelocity += 10;
        }

        telemetry.addLine("Gamepad2.dpad_down -> decrease velocity by 10");
        if (gamepad2.dpad_down) {
            dualVelocity-= 10;
        }

        telemetry.update();
        update();



    }
}

