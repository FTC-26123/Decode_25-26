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

        PID_backward(44,0.5);

        gate.setPosition(0.5);

        shooter.setVelocity(1900);
        runShooter(2025);
        sleep(3000);

        gate.setPosition(0.8);
        runShooter(2025);
        sleep(1500);

        gate.setPosition(0.35);
        runShooter(2025);
        windmill.setPower(1);
        sleep(1500);

        gate.setPosition(0.8);
        windmill.setPower(0);
        runShooter(2025);
        sleep(1500);

        gate.setPosition(0.35);
        runShooter(2025);
        windmill.setPower(1);
        sleep(1500);

        gate.setPosition(0.8);
        runShooter(2025);
        sleep(1500);

        gate.setPosition(0.35);
        shooter.setVelocity(0);
        sleep(1000);

        PID_rotateRight(90,0.7);

        PID_forward(12,0.5);

    }
}
