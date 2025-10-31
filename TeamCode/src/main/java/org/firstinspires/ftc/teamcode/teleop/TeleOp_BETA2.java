package org.firstinspires.ftc.teamcode.teleop;


import static java.lang.Thread.sleep;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "TeleOp_BETA without limelight id")
public class TeleOp_BETA2 extends OpMode {
    //Initializing and declaring all variables/motors
    public final float MOTOR_MULTIPLIER_PERCENTAGE_CAP = 0.55F;
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    public DcMotorEx shooter;

    public DcMotor windmill;
    public DcMotor intake;
    public Servo gate;
    public DigitalChannel touch;
    public Servo light1;

    public float frontLeftMotorSpeed = 0;
    public float frontRightMotorSpeed = 0;
    public float backLeftMotorSpeed = 0;
    public float backRightMotorSpeed = 0;

    public double launchPower;
    public boolean launchStarted = false;

    private Limelight3A limelight;

    // Limelight Variable
    // Limelight Constants
    final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 55;
    final double LIMELIGHT_LENS_HEIGHT_INCHES = 14;
    final double goalHeight = 40;
    final double APRILTAG_HEIGHT = 25;

    double launchAngleDegrees = 45.0;
    double launchHeight = 10.0;
    double fudgeFactor = 0.3;

    double finalvelocity = 0;

    double actualvelocity = 0;


    //lights constant
    private ElapsedTime runtime = new ElapsedTime();

    public NormalizedColorSensor colorSensor;


    public void update() {
//     Robot Motor Power Limits
        frontLeftMotor.setPower(frontLeftMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        frontRightMotor.setPower(frontRightMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        backLeftMotor.setPower(backLeftMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        backRightMotor.setPower(backRightMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);
    }

    @Override
    public void init() {
        // Motors & Servos
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        shooter = hardwareMap.get(DcMotorEx.class, "launcher");
        windmill = hardwareMap.get(DcMotor.class, "windmill");
        intake = hardwareMap.get(DcMotor.class, "intake");
        gate = hardwareMap.get(Servo.class, "gate");

        light1 = hardwareMap.get(Servo.class, "light1");

        touch = hardwareMap.get(DigitalChannel.class, "touch");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "revColorV3");
        colorSensor.setGain(7);

        launchStarted = false;

    }

    public boolean gateIsTouching(boolean addGateTelemetry) {
        if (addGateTelemetry) {
            telemetry.addLine("Gate Is Touching");
        } else {
            telemetry.update();
        }

        return (touch.getState());

    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        // Color Sensing Code
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        int col = colors.toColor();
        double hue = JavaUtil.colorToHue(col);

        telemetry.addData("Detected Hue", hue);
        telemetry.addData("Detected Color", col);

        if (hue > 90 && hue < 180) {
            telemetry.addLine("Detected Color: Green");
            light1.setPosition(0.47);
        } else if (hue > 215 && hue < 255) {
            telemetry.addLine("Detected Color: Purple");
            light1.setPosition(0.67);
        } else {
            light1.setPosition(0);
        }

        double time = runtime.seconds();
        // Check for the distance
        LLResult result = limelight.getLatestResult();

        //limelight
        double tx = 0;
        double ty = 0;
        Pose3D botpose = null;
        double distance = 0;
        // modify :
        double shooterVelocity = -1;
        double k = fudgeFactor;
        double flywheel = 1.89;

        if (result != null && result.isValid()) {
            tx = result.getTx();
            ty = result.getTy();
            botpose = result.getBotpose();

            // --- Distance Calculation ---
            double angleToGoalDegrees = LIMELIGHT_MOUNT_ANGLE_DEGREES + ty;
            double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
            distance = (APRILTAG_HEIGHT - LIMELIGHT_LENS_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);

            telemetry.addData("distance", distance);

            double g = 386.0; // gravity in in/s^2
            double theta = Math.toRadians(launchAngleDegrees);
            double y = goalHeight - launchHeight;

            double denominator = 2 * Math.pow(Math.cos(theta), 2) * (distance * Math.tan(theta) - y);
            if (denominator <= 0)
                telemetry.addLine("Too close");

            double velocity = Math.sqrt((g * distance * distance) / denominator) * (1 + k);

            finalvelocity = velocity / flywheel;

            shooter.setVelocity(finalvelocity);

        }

        if (result == null || !result.isValid()) {
            // Show red even when april tag is not visible to bot
            telemetry.addLine("No Target Found");
        }

        actualvelocity = finalvelocity;


        // Movement w/ Joysticks
        frontLeftMotorSpeed = 0;
        frontRightMotorSpeed = 0;
        backLeftMotorSpeed = 0;
        backRightMotorSpeed = 0;

        float left_stick_x = gamepad1.left_stick_x;
        float left_stick_y = gamepad1.left_stick_y;
        float right_stick_x = gamepad1.right_stick_x;

//        Forward/Backward Movement
        if (left_stick_y != 0) {
            frontLeftMotorSpeed = -left_stick_y;
            frontRightMotorSpeed = -left_stick_y;
            backLeftMotorSpeed = -left_stick_y;
            backRightMotorSpeed = -left_stick_y;
        }

//        Lateral Movement
        if (left_stick_x != 0) {
            frontLeftMotorSpeed += left_stick_x;
            frontRightMotorSpeed -= left_stick_x;
            backLeftMotorSpeed -= left_stick_x;
            backRightMotorSpeed += left_stick_x;
        }

//        Rotation
        if (right_stick_x != 0) {
            frontLeftMotorSpeed += right_stick_x;
            backLeftMotorSpeed += right_stick_x;
            frontRightMotorSpeed -= right_stick_x;
            backRightMotorSpeed -= right_stick_x;
        }

        launchPower = (gamepad2.right_trigger * 6000);

        if (launchPower <= 3000) {
            launchPower = 3000;
        }
        if (gamepad2.right_trigger > 0.35) {
            launchStarted = true;
        }
        if (launchStarted) {
            shooter.setVelocity(launchPower);
        }
        if (gamepad2.right_bumper) {
            shooter.setVelocity(0);
            launchStarted = false;
        }



        if (gamepad2.left_trigger > 0.5) {
            intake.setPower(-0.5);
            windmill.setPower(1);
        } else if (gamepad2.back) {
            intake.setPower(0.5);
            windmill.setPower(-1);
        } else if (gamepad2.left_bumper) {
            intake.setPower(0);
            windmill.setPower(0);
        }

        if (gamepad2.b) {
            gate.setPosition(0.05);
        } else if (gamepad2.x) {
            gate.setPosition(0.8);
        } else if (gamepad2.y) {
            gate.setPosition(0.5);
        }

        telemetry.setMsTransmissionInterval(30);

        telemetry.update();
        update();

    }
}


