package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "newRobotTesting")
public class newRobotTesting extends OpMode {

    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    public DcMotorEx launcherRight;
    public DcMotor index;
    public DcMotor intake;
    public DcMotorEx launcherLeft;
    long dualVelocity = 2250;

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
        launcherLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        index.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

        telemetry.addLine("Gamepad2.a -> intake");
        if (gamepad2.a) {
            intake.setPower(1);
        }

        telemetry.addLine("Gamepad2.b -> index / windmill");
        if (gamepad2.b) {
            index.setPower(1);
        }

        telemetry.addLine("Gamepad2.y -> left launcher");
        telemetry.addData("Launch Power (left)", launcherLeft.getVelocity());
        if (gamepad2.y) {
            launcherLeft.setVelocity(dualVelocity);
        }

        telemetry.addLine("Gamepad2.x -> right launcher");
        telemetry.addData("Launch Power (right)", launcherRight.getVelocity());
        if (gamepad2.x) {
            launcherRight.setVelocity(dualVelocity);
        }

        telemetry.addLine("Gamepad2.ps -> all off");
        if (gamepad2.ps) {
            intake.setPower(0);
            launcherLeft.setVelocity(0);
            launcherRight.setVelocity(0);
            index.setPower(0);
        }

        telemetry.addLine("Gamepad2.start -> reverse motor");
        if (gamepad2.start) {
            index.setDirection(DcMotorSimple.Direction.FORWARD);
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        telemetry.addLine("Gamepad2.back -> forward motor");
        if (gamepad2.back) {
            index.setDirection(DcMotorSimple.Direction.REVERSE);
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        telemetry.addLine("Gamepad2.dpad_up -> increase velocity by 10");
        if (gamepad2.dpad_up) {
            dualVelocity = dualVelocity + 10;
        }

        telemetry.addLine("Gamepad2.dpad_down -> decrease velocity by 10");
        if (gamepad2.dpad_down) {
            dualVelocity = dualVelocity - 10;
        }

        telemetry.addData("Target Velocity", dualVelocity);

        telemetry.update();





    }
}
