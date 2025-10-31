package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import static org.firstinspires.ftc.teamcode.Commons.PID_backward;
import static org.firstinspires.ftc.teamcode.Commons.PID_forward;
import static org.firstinspires.ftc.teamcode.Commons.PID_rotateLeft;
import static org.firstinspires.ftc.teamcode.Commons.PID_rotateRight;
import static org.firstinspires.ftc.teamcode.Commons.backLeftMotor;
import static org.firstinspires.ftc.teamcode.Commons.backRightMotor;
import static org.firstinspires.ftc.teamcode.Commons.frontLeftMotor;
import static org.firstinspires.ftc.teamcode.Commons.frontRightMotor;
import static org.firstinspires.ftc.teamcode.Commons.lateralLeft;
import static org.firstinspires.ftc.teamcode.Commons.colorSensor;
import static org.firstinspires.ftc.teamcode.Commons.gate;
import static org.firstinspires.ftc.teamcode.Commons.intake;
import static org.firstinspires.ftc.teamcode.Commons.limelight;
import static org.firstinspires.ftc.teamcode.Commons.opModeIsActive;
import static org.firstinspires.ftc.teamcode.Commons.shooter;
import static org.firstinspires.ftc.teamcode.Commons.startLateralLeft;
import static org.firstinspires.ftc.teamcode.Commons.windmill;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.Commons;

import java.util.Timer;

@Autonomous(name = "redWall")
public class redWall extends LinearOpMode {



    public void runOpMode() throws InterruptedException {
        Commons.init(hardwareMap, this::opModeIsActive, telemetry);

        telemetry.setMsTransmissionInterval(100);

        waitForStart();

        PID_forward(55, 0.70);

        PID_rotateRight(90, 0.40);

        intake.setPower(-0.5);
        windmill.setPower(1);

        PID_forward(12, 0.4);

        intake.setPower(0);
        windmill.setPower(0);

        PID_backward(29, 0.75);
        sleep(1000);

        PID_rotateLeft(90, 0.5);
        frontRightMotor.setPower(0.3);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setPower(0.3);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setPower(0.3);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setPower(0.3);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        sleep(2000);
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        sleep(1000);
        windmill.setPower(0.5);
        shooter.setPower(0.7);
        sleep(3000);
        gate.setPosition(0.05);
        windmill.setPower(0);
        shooter.setPower(0.7);
        sleep(1500);
//        windmill.setPower(0);
        shooter.setPower(0);
    }
}