package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.Commons.PID_forward;
import static org.firstinspires.ftc.teamcode.Commons.PID_rotateLeft;

import org.firstinspires.ftc.teamcode.Commons;

@Autonomous(name = "goalWall_blue_MoveForward")
public class goalWall_blue_MoveForward extends LinearOpMode {


    public void runOpMode() throws InterruptedException {
        Commons.init(hardwareMap, this::opModeIsActive, telemetry);

        PID_rotateLeft(75, 0.25);

        PID_forward(20, 0.25);

        sleep(5000);
    }
}
