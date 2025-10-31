package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.Commons.PID_forward;


import org.firstinspires.ftc.teamcode.Commons;

    @Autonomous(name = "wallMoveForward")
    public class moveForward extends LinearOpMode {


        public void runOpMode() throws InterruptedException {
            Commons.init(hardwareMap, this::opModeIsActive, telemetry);

            PID_forward(10, 0.25);
            sleep(5000);
        }
    }
