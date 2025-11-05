package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.Commons.*;

import org.firstinspires.ftc.teamcode.Commons;

@Autonomous(name="goalWall_red_simple")
public class goalwall_red_simple extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Commons.init(hardwareMap, this::opModeIsActive, telemetry);
        telemetry.setMsTransmissionInterval(35);
        waitForStart();

//        Use time based lateral movement until PID works

//        Starts the shooter to save time and sets gate to neutral
        runShooter(1990);
        gate.setPosition(0.35);

//        Goes backward to shoot
        PID_backward(44,0.67);

        sleep(250);

//        Shoots
        gate.setPosition(0.8);
        runShooter(1990);
        sleep(1500);

//        Resets gate position
        gate.setPosition(0.35);
        runShooter(1990);
        windmill.setPower(1);
        sleep(1500);

//        Shoots
        gate.setPosition(0.8);
        windmill.setPower(0);
        runShooter(1990);
        sleep(1500);

//        Resets gate position
        gate.setPosition(0.35);
        runShooter(1990);
        windmill.setPower(1);
        sleep(1500);

//        Shoots
        gate.setPosition(0.8);
        runShooter(1990);
        sleep(1500);

//        Resets gate position
        gate.setPosition(0.35);
        shooter.setVelocity(0);
        sleep(1000);

//        Rotates right to pick up more artifacts
        PID_rotateRight(49,0.4);

//        Lateral to avoid the center
//        lateralRight(6,0.4);

        startLateralRight(0.4);
        sleep(1250);
        stopMotorsLateralRight();

//        Prepares Intake
        intake.setPower(-0.8);
        windmill.setPower(1);

//        Forward to pick up artifacts
        PID_forward(35,0.4);

//        Intake off + waits
        intake.setPower(0);
        sleep(1000);


//        lateralLeft(6,0.4);

//        startLateralLeft(0.4);
//        sleep(1250);
//        stopMotorsLateralLeft();

//        Windmill off
        windmill.setPower(0);

//        Rotate left to shoot near the gate
        PID_rotateLeft(80,0.4);

//        Same as last setup
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
