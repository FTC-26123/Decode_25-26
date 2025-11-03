package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static org.firstinspires.ftc.teamcode.Commons.*;

import org.firstinspires.ftc.teamcode.Commons;

@Autonomous(name="goalWall_red_simple")
public class goalwall_red_simple extends LinearOpMode {

    public void runShooter(double velocity) {
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setVelocity(velocity);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Commons.init(hardwareMap, this::opModeIsActive, telemetry);
        telemetry.setMsTransmissionInterval(35);
        waitForStart();

//        Use time based lateral movement until PID works

        runShooter(1990);
        gate.setPosition(0.35);

        PID_backward(44,0.67);

        sleep(800);

        gate.setPosition(0.8);
        runShooter(1990);
        sleep(1500);

        gate.setPosition(0.35);
        runShooter(1990);
        windmill.setPower(1);
        sleep(1500);

        gate.setPosition(0.8);
        windmill.setPower(0);
        runShooter(1990);
        sleep(1500);

        gate.setPosition(0.35);
        runShooter(1990);
        windmill.setPower(1);
        sleep(1500);

        gate.setPosition(0.8);
        runShooter(1990);
        sleep(1500);

        gate.setPosition(0.35);
        shooter.setVelocity(0);
        sleep(1000);

        PID_rotateRight(49,0.4);

//        lateralRight(6,0.4);

        startLateralRight(0.4);
        sleep(1250);
        stopMotorsLateralRight();

        intake.setPower(-0.8);
        windmill.setPower(1);

        PID_forward(35,0.4);

        intake.setPower(0);

        sleep(1000);

//        lateralLeft(6,0.4);

        startLateralLeft(0.4);
        sleep(1250);
        stopMotorsLateralLeft();

        windmill.setPower(0);

        PID_rotateLeft(80,0.4);

        shooter.setVelocity(1900);
        runShooter(1990);
        sleep(3000);

        gate.setPosition(0.8);
        runShooter(1990);
        sleep(1500);


        gate.setPosition(0.35);
        runShooter(1990);
        windmill.setPower(1);
        sleep(1500);

        gate.setPosition(0.8);
        windmill.setPower(0);
        runShooter(1990);
        sleep(1500);

        gate.setPosition(0.35);
        runShooter(1990);
        windmill.setPower(1);
        sleep(1500);

        gate.setPosition(0.8);
        runShooter(1990);
        sleep(1500);

        gate.setPosition(0.35);
        shooter.setVelocity(0);
        sleep(1000);

    }
}
