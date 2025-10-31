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

        PID_backward(44,0.5);

        gate.setPosition(0.5);

        shooter.setVelocity(5250);
        sleep(5000);

        gate.setPosition(0.05);
        shooter.setVelocity(5250);
        sleep(1500);

        gate.setPosition(0.5);
        shooter.setVelocity(5250);
        windmill.setPower(1);
        sleep(1500);

        gate.setPosition(0.05);
        shooter.setVelocity(5250);
        sleep(1500);

        gate.setPosition(0.5);
        shooter.setVelocity(5250);
        windmill.setPower(1);
        sleep(1500);

        gate.setPosition(0.05);
        shooter.setVelocity(5250);
        sleep(1500);

        gate.setPosition(0.5);
        shooter.setVelocity(0);
        sleep(1000);

        PID_forward(30,0.2);

    }
}
