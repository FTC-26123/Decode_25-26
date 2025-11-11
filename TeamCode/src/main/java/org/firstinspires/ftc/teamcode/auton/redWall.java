package org.firstinspires.ftc.teamcode.auton;

import static java.lang.Thread.sleep;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
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

@Autonomous(name = "RedWall", group = "Autonomous")
@Configurable
public class redwall extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private int pathState;

    private Timer pathTimer;
    private Paths paths;

    public DcMotorEx shooter;

    public DcMotor windmill;
    public DcMotor intake;
    public Servo gate;

    public Servo light1;
    
    final int shooterVelocity = 1925;
    final float NEUTRAL_POS = 0.35f;
    final float SHOOT_POS = 0.80f;

    ElapsedTime timer = new ElapsedTime();

    public void set_timer(int milliseconds) {
        timer.reset();
        while (timer.milliseconds() <= milliseconds) {}
    }

    @Override
    public void init() {

        shooter = hardwareMap.get(DcMotorEx.class, "launcher");
        windmill = hardwareMap.get(DcMotor.class, "windmill");
        intake = hardwareMap.get(DcMotor.class, "intake");
        gate = hardwareMap.get(Servo.class, "gate");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        light1 = hardwareMap.get(Servo.class, "light1");

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(117.47368421052632, 128.65263157894736, Math.toRadians(42)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
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

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    /** ---------------- PATH DEFINITIONS ---------------- **/
    public static class Paths {
        public PathChain Shoot1set, go2ndset, Intake2ndset, Shoot2ndset, go3rdset, Intake3rdset;

        public Paths(Follower follower) {
            Shoot1set = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(123.404, 131.508), new Pose(88.291, 98.589))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(43))
                    .build();

            go2ndset = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(88.291, 98.589),
                                    new Pose(86.096, 96.394),
                                    new Pose(97.407, 90.823)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))
                    .build();

            Intake2ndset = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(97.407, 90.823), new Pose(131.508, 90.654))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Shoot2ndset = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(131.508, 90.654), new Pose(88.122, 98.589))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))
                    .build();

            go3rdset = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(88.122, 98.589),
                                    new Pose(84.914, 66.851),
                                    new Pose(99.939, 67.020)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(4), Math.toRadians(0))
                    .build();

            Intake3rdset = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(99.939, 67.020), new Pose(131.170, 66.851))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

        }
    }

    /** ---------------- STATE MACHINE ---------------- **/
    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {

            case 0: // go to shooting position
                light1.setPosition(0.29);
                shooter.setVelocity(shooterVelocity);
                follower.followPath(paths.Shoot1set);
                setPathState(1);


                break;
            case 1: //shoot 1st set
                if(!follower.isBusy()) {
                    light1.setPosition(0.333);
                    // Ball #1
                    gate.setPosition(SHOOT_POS);
                    set_timer(2500);
                    gate.setPosition(NEUTRAL_POS);

                    windmill.setPower(0.75);
                    set_timer(1500);
                    windmill.setPower(0);
                    // Ball #2
                    gate.setPosition(SHOOT_POS);
                    set_timer(1500);
                    gate.setPosition(NEUTRAL_POS);

                    windmill.setPower(0.75);
                    set_timer(1250);
                    windmill.setPower(0);
                    // Ball #3
                    gate.setPosition(SHOOT_POS);
                    set_timer(800);
                    gate.setPosition(NEUTRAL_POS);
                    setPathState(2);
                }
                break;
            case 2: // get ready to intake
                if (!follower.isBusy()) {
                    light1.setPosition(0.388);
                    intake.setPower(-0.85);
                    windmill.setPower(1.0);
                    shooter.setVelocity(1200);
                    follower.followPath(paths.go2ndset, 0.8, false);
                    setPathState(3);
                }
                break;

            case 3: //intake
                if (!follower.isBusy()) {
                    light1.setPosition(0.444);
                    follower.followPath(paths.Intake2ndset, 0.5, true);
                    setPathState(4);
                }

            case 4:
                if (!follower.isBusy()) {
                    light1.setPosition(0.500);
                    shooter.setVelocity(shooterVelocity);
                    intake.setPower(0.0);
                    windmill.setPower(0.4);
                    follower.followPath(paths.Shoot2ndset);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    light1.setPosition(0.555);
                    // Ball #1
                    gate.setPosition(SHOOT_POS);
                    set_timer(1500);
                    gate.setPosition(NEUTRAL_POS);

                    windmill.setPower(0.75);
                    set_timer(1500);
                    windmill.setPower(0);
                    // Ball #2
                    gate.setPosition(SHOOT_POS);
                    set_timer(1500);
                    gate.setPosition(NEUTRAL_POS);

                    windmill.setPower(0.75);
                    set_timer(1500);
                    windmill.setPower(0);
                    // Ball #3
                    gate.setPosition(SHOOT_POS);
                    set_timer(1500);
                    gate.setPosition(NEUTRAL_POS);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    light1.setPosition(0.611);
                    shooter.setVelocity(0);
                    intake.setPower(-0.85);
                    windmill.setPower(1);
                    follower.followPath(paths.go3rdset, 0.8, false);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    light1.setPosition(0.667);
                    follower.followPath(paths.Intake3rdset, 0.5, true);
                    setPathState(-1); //stop
                }

//            case 8:
//                if (!follower.isBusy()) {
//                    light1.setPosition(0.611);
//                    shooter.setVelocity(1900);
//                    intake.setPower(0);
//                    windmill.setPower(0);
//                    follower.followPath(paths.Shoot3rdset);
//                    setPathState(7);
//                }
//                break;
//
//            case 7:
//                if (!follower.isBusy()) {
//                    light1.setPosition(0.667);
//                    // Ball #1
//                    gate.setPosition(SHOOT_POS);
//                    set_timer(1500);
//                    gate.setPosition(NEUTRAL_POS);
//
//                    windmill.setPower(0.75);
//                    set_timer(1500);
//                    windmill.setPower(0);
//                    // Ball #2
//                    gate.setPosition(SHOOT_POS);
//                    set_timer(1500);
//                    gate.setPosition(NEUTRAL_POS);
//
//                    windmill.setPower(0.75);
//                    set_timer(1500);
//                    windmill.setPower(0);
//                    // Ball #3
//                    gate.setPosition(SHOOT_POS);
//                    set_timer(1500);
//                    gate.setPosition(NEUTRAL_POS);
//                    intake.setPower(-0.8);
//                    windmill.setPower(0.75);
//                    set_timer(500);
//                    setPathState(8);
//                }
//                break;
//
//            case 8:
//                if (!follower.isBusy()) {
//                    light1.setPosition(0.722);
//                    shooter.setVelocity(0);
//                    intake.setPower(0);
//                    windmill.setPower(0);
//                    follower.followPath(paths.rankingPoints);
//                    setPathState(-1); // stop
//                }
//                break;
        }
    }



    private void setPathState(int s) {
        pathState = s;
        pathTimer.resetTimer();
    }
}