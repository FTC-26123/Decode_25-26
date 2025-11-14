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
public class bluewall extends OpMode {

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

    final int shooterVelocity = 1930;
    final float NEUTRAL_POS = 0.35f;
    final float SHOOT_POS = 0.80f;

    ElapsedTime timer = new ElapsedTime();

    public void wait(int milliseconds) {
        timer.reset();
        while (true) {if (timer.milliseconds() >= milliseconds) break;}
    }

    public void velocityCheck(int ms, int velocity) {
        while (shooter.getVelocity()<velocity) {wait(ms);}
    }

    public static double cc(double x) { // converting coordinates
        return 72-(x-72);
    }

    public static double d(double degrees) { //converting degrees
        return(180-degrees);
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
        follower.setStartingPose(new Pose(119.353, 127.456, Math.toRadians(42)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        telemetry.setMsTransmissionInterval(20);

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
        telemetry.addData("Shooter Velocity", shooter.getVelocity());
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);



        telemetry.update();
    }

    /** ---------------- PATH DEFINITIONS ---------------- **/
    public static class Paths {
        public PathChain Shoot1set, go2ndset, Intake2ndset, Shoot2ndset, go3rdset, Intake3rdset;

        public Paths(Follower follower) {
            Shoot1set = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(cc(119.353), 127.456), new Pose(cc(89.810+3), 98.420+3))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(d(42)), Math.toRadians(d(45)))
                    .build();

            go2ndset = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(cc(89.810+3), 98.420+3),
                                    new Pose(cc(83.395), 87.109),
                                    new Pose(cc( 99.433), 82.382)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(d(45)), Math.toRadians(d(0)))
                    .build();

            Intake2ndset = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(cc(99.433-1), 82.382), new Pose(cc(126.106), 81.876))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(d(0)), Math.toRadians(d(0)))
                    .build();

            Shoot2ndset = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(cc(126.106), 81.876), new Pose(cc((89.810+3)), 98.420+3))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(d(0)), Math.toRadians(d(42)))
                    .build();

            go3rdset = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(cc(89.810+3), 98.420+3),
                                    new Pose(cc(86.096), 61.111),
                                    new Pose(cc(99.939), 57.566)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(d(42)), Math.toRadians(d(0)))
                    .build();

            Intake3rdset = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(cc(99.939+2), 57.566-0), new Pose(cc(125.599), 57.735))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(d(0)), Math.toRadians(d(0)))
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
                    velocityCheck(5, 1920);
                    gate.setPosition(SHOOT_POS);
                    wait(1500);
                    shooter.setVelocity(shooterVelocity+20);
                    gate.setPosition(NEUTRAL_POS);

                    windmill.setPower(0.75);
                    wait(1500);
                    windmill.setPower(0);
                    // Ball #2
                    velocityCheck(5, 1920);
                    gate.setPosition(SHOOT_POS);
                    wait(1000);
                    gate.setPosition(NEUTRAL_POS);

                    windmill.setPower(0.75);
                    wait(1250);
                    windmill.setPower(0);
                    shooter.setVelocity(shooterVelocity);
                    // Ball #3
                    velocityCheck(5, 1920);
                    gate.setPosition(SHOOT_POS);
                    wait(1000);
                    gate.setPosition(NEUTRAL_POS);
                    setPathState(2);
                }
                break;
            case 2: // get ready to intake
                if (!follower.isBusy()) {
                    light1.setPosition(0.388);
                    intake.setPower(-1);
                    windmill.setPower(1.0);
                    shooter.setVelocity(1200);
                    follower.followPath(paths.go2ndset, 0.8, false);
                    setPathState(3);
                }
                break;

            case 3: //intake
                if (!follower.isBusy()) {
                    light1.setPosition(0.444);
                    follower.followPath(paths.Intake2ndset, 0.25, true);
                    setPathState(4);
                }

            case 4:
                if (!follower.isBusy()) {
                    light1.setPosition(0.500);
                    shooter.setVelocity(shooterVelocity+15);
                    intake.setPower(-0.5);
                    windmill.setPower(0.5);
                    follower.followPath(paths.Shoot2ndset);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    light1.setPosition(0.555);
                    intake.setPower(0);
                    // Ball #1
                    velocityCheck(5, 1930);
                    gate.setPosition(SHOOT_POS);
                    wait(1500);
                    gate.setPosition(NEUTRAL_POS);

                    windmill.setPower(0.75);
                    shooter.setVelocity(shooterVelocity+35);
                    wait(1500);
                    windmill.setPower(0);
                    // Ball #2
                    velocityCheck(5, 1935);
                    gate.setPosition(SHOOT_POS);
                    wait(1500);
                    gate.setPosition(NEUTRAL_POS);

                    windmill.setPower(0.75);
                    shooter.setVelocity(shooterVelocity+15);
                    wait(1500);
                    windmill.setPower(0);
                    // Ball #3
                    velocityCheck(5, 1930);
                    gate.setPosition(SHOOT_POS);
                    wait(1500);
                    gate.setPosition(NEUTRAL_POS);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    light1.setPosition(0.611);
                    shooter.setVelocity(0);
                    intake.setPower(1); // outtake if ball is stuck
                    windmill.setPower(-1); // outtake if ball is stuck
                    follower.followPath(paths.go3rdset, 0.8, false);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    light1.setPosition(0.667);
                    intake.setPower(-1); //intake on
                    windmill.setPower(1); //intake on
                    follower.followPath(paths.Intake3rdset, 0.25, true);
                    wait(1000);
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
//                    wait(1500);
//                    gate.setPosition(NEUTRAL_POS);
//
//                    windmill.setPower(0.75);
//                    wait(1500);
//                    windmill.setPower(0);
//                    // Ball #2
//                    gate.setPosition(SHOOT_POS);
//                    wait(1500);
//                    gate.setPosition(NEUTRAL_POS);
//
//                    windmill.setPower(0.75);
//                    wait(1500);
//                    windmill.setPower(0);
//                    // Ball #3
//                    gate.setPosition(SHOOT_POS);
//                    wait(1500);
//                    gate.setPosition(NEUTRAL_POS);
//                    intake.setPower(-0.8);
//                    windmill.setPower(0.75);
//                    wait(500);
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