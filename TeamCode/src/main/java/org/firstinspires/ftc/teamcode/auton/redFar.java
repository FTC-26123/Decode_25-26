package org.firstinspires.ftc.teamcode.auton;

import static java.lang.Thread.sleep;


import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;

@Autonomous(name = "redFar", group = "Autonomous")
public class redFar extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private int pathState;
    private Timer pathTimer;
    private Paths paths;
    public DcMotorEx launcher;
    public DcMotor index;
    public DcMotor intake;
    public Servo light1;
    public Servo gate;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime velTim = new ElapsedTime();

    public void wait(int milliseconds) {
        timer.reset();
        while (true) {
            if (timer.milliseconds() > milliseconds) {break;}
        }
    }

    public void velCheck(int minVelocity) {
        while(launcher.getVelocity() < minVelocity) wait(10);
    }

    @Override
    public void init() {

        light1 = hardwareMap.get(Servo.class, "light");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        index = hardwareMap.get(DcMotor.class, "windmill");
        intake = hardwareMap.get(DcMotor.class, "intake");

        gate = hardwareMap.get(Servo.class, "gate");

        launcher.setDirection(DcMotorSimple.Direction.REVERSE);;
        index.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);


        pathTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(80.822, 8.340, Math.toRadians(90)));

        paths = new Paths(follower);



    }

    @Override
    public void start() {
        pathState = 0;
        pathTimer.resetTimer();
        timer.reset();
    }

    @Override
    public void loop() {
        follower.update();
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

    }

    /** ---------------- PATH DEFINITIONS ---------------- **/

    public static class Paths {
        public PathChain shoot1st;
        public PathChain intake2nd;
        public PathChain shoot2nd;
        public PathChain intake3rd;
        public PathChain shoot3rd;
        public PathChain goToGate;

        public Paths(Follower follower) {
            shoot1st = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(80.822, 8.340),

                                    new Pose(87.282, 18.489)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(63))

                    .build();

            intake2nd = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(87.282, 18.489),
                                    new Pose(87.311, 37.761),
                                    new Pose(82.271, 36.486),
                                    new Pose(126.102, 35.702)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(63), Math.toRadians(0))

                    .build();

            shoot2nd = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(126.102, 35.702),

                                    new Pose(87.325, 18.360)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(63))

                    .build();

            intake3rd = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(87.325, 18.360),
                                    new Pose(74.478, 64.960),
                                    new Pose(124.495, 59.577)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(63), Math.toRadians(0))

                    .build();

            shoot3rd = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(124.495, 59.577),

                                    new Pose(87.226, 18.362)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(63))

                    .build();

            goToGate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(87.226, 18.362),
                                    new Pose(114.068, 73.149),
                                    new Pose(122.509, 70.973)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(63), Math.toRadians(0))

                    .build();
        }
    }


    /** ---------------- STATE MACHINE ---------------- **/
    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {

            case 0:
                if(!follower.isBusy()) {

                    launcher.setVelocity(1470);
                    intake.setPower(0.7);
                    follower.followPath(paths.shoot1st);
                    setPathState(1);
                }
                break;

            case 1:
                if(!follower.isBusy()) {

                    intake.setPower(0.8);
                    gate.setPosition(0.27);

                    velCheck(1420);
                    index.setPower(0.7);
                    wait(1000);
                    index.setPower(0);
                    wait(300);

                    velCheck(1420);
                    index.setPower(0.7);
                    wait(1000);
                    index.setPower(0);
                    wait(300);

                    velCheck(1420);
                    index.setPower(0.7);
                    wait(1000);
                    index.setPower(0);
                    wait(300);

                    gate.setPosition(0.50);
                    setPathState(2);

                }
                break;

            case 2:
                if(!follower.isBusy()) {
                    intake.setPower(1);
                    index.setPower(0.5);
                    follower.followPath(paths.intake2nd, 0.65, true);
                    launcher.setVelocity(1470);
                    setPathState(3);
                }
                break;

            case 3:
                if(!follower.isBusy()) {
                    intake.setPower(0.5);
                    index.setPower(0);
                    follower.followPath(paths.shoot2nd);
                    setPathState(4);
                }
                break;

            case 4:
                if(!follower.isBusy()) {

                    gate.setPosition(0.27);
                    velCheck(1420);
                    index.setPower(0.7);
                    wait(1000);
                    index.setPower(0);
                    wait(300);

                    velCheck(1420);
                    index.setPower(0.7);
                    wait(1000);
                    index.setPower(0);
                    wait(300);

                    velCheck(1420);
                    index.setPower(0.7);
                    wait(1000);
                    index.setPower(0);
                    wait(300);
                    gate.setPosition(0.50);

                    setPathState(5);
                }
                break;


            case 5:
                if(!follower.isBusy()) {
                    intake.setPower(1);
                    index.setPower(0.5);
                    follower.followPath(paths.intake3rd,0.65,true);
                    setPathState(6);
                }
                break;

            case 6:
                if(!follower.isBusy()) {
                    intake.setPower(0.5);
                    index.setPower(0);
                    follower.followPath(paths.shoot3rd);
                    setPathState(7);
                }
                break;

            case 7:
                if(!follower.isBusy()) {

                    gate.setPosition(0.27);
                    velCheck(1420);
                    index.setPower(0.7);
                    wait(1000);
                    index.setPower(0);
                    wait(300);

                    velCheck(1420);
                    index.setPower(0.7);
                    wait(1000);
                    index.setPower(0);
                    wait(300);

                    velCheck(1420);
                    index.setPower(0.7);
                    wait(1000);
                    index.setPower(0);
                    wait(300);
                    gate.setPosition(0.50);

                    setPathState(8);
                }
                break;

            case 8:
                if(!follower.isBusy()) {
                    launcher.setVelocity(0);
                    index.setPower(0);
                    intake.setPower(0);
                    follower.followPath(paths.goToGate, 0.65, true);
                    setPathState(-1);
                }

                break;

        }
    }

    private void setPathState(int s) {
        pathState = s;
        pathTimer.resetTimer();
    }
}
