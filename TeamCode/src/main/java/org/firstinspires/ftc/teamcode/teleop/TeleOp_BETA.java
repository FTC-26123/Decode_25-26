package org.firstinspires.ftc.teamcode.teleop;


import static java.lang.Thread.sleep;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "TeleOp_BETA")
public class TeleOp_BETA extends OpMode {
//Initializing and declaring all variables/motors
    public final float MOTOR_MULTIPLIER_PERCENTAGE_CAP = 0.55F;
    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    public DcMotor shooter;
    public DcMotor windmill;
    public DcMotor intake;
    public Servo gate;
    public float frontLeftMotorSpeed = 0;
    public float frontRightMotorSpeed = 0;
    public float backLeftMotorSpeed = 0;
    public float backRightMotorSpeed = 0;
    String Pattern;
//    private Limelight3A limelight;

    // Limelight Variable
    // Limelight Constants
    private final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 135;
    private final double LIMELIGHT_LENS_HEIGHT_INCHES = 15;
    private final double GOAL_HEIGHT_INCHES = 40;
    private final double APRILTAG_HEIGHT = 25;

    //lights constant
//    private Servo blinkin;
    private ElapsedTime runtime = new ElapsedTime();

    public NormalizedColorSensor colorSensor;

    public boolean detectingPurple;
    public boolean detectingBlue;



    //lower- less chance it is detected
    //higher- higher chance it is detected
    //blue and purple should be a little bit more than the others
    //tolerance_blue2 is the tolerance for * purple * # (purple is x5 the other colors)
    public final double tolerance_red = 1.5;
    public final double tolerance_blue = 3.5;
    public final double tolerance_blue2 = 10;
    public final double tolerance_green = 1.3;

    public final double tolerance_purple = 2.3;

    public void update(){
//     Robot Motor Power Limits
        frontLeftMotor.setPower(frontLeftMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        frontRightMotor.setPower(frontRightMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        backLeftMotor.setPower(backLeftMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        backRightMotor.setPower(backRightMotorSpeed * MOTOR_MULTIPLIER_PERCENTAGE_CAP);
    }

    @Override
    public void init(){
        // Motors & Servos
        frontLeftMotor=hardwareMap.get(DcMotor.class,"frontLeftMotor");
        frontRightMotor=hardwareMap.get(DcMotor.class,"frontRightMotor");
        backLeftMotor=hardwareMap.get(DcMotor.class,"backLeftMotor");
        backRightMotor=hardwareMap.get(DcMotor.class,"backRightMotor");
        shooter=hardwareMap.get(DcMotor.class,"launcher");
        windmill=hardwareMap.get(DcMotor.class,"windmill");
        intake=hardwareMap.get(DcMotor.class,"intake");
        gate=hardwareMap.get(Servo.class,"gate");
        // Limelight
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(0);
//        limelight.start();
        // Lights
//        blinkin = hardwareMap.get(Servo.class, "blinkin");
        // Color Sensor
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "revColorV3");
        colorSensor.setGain(7);
        detectingBlue = false;
        detectingPurple = false;

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void start(){
        runtime.reset();
    }

    @Override
    public void loop() {
        // Color Sensing Code
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float normRed, normGreen, normBlue, normPurple;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;
        normPurple = (colors.red + colors.blue) / colors.alpha;

        double time = runtime.seconds();
        // Check for the distance
//        LLResult result = limelight.getLatestResult();

        //limelight
        double tx = 0;
        double ty = 0;
        Pose3D botpose = null;
        double distanceInches = 0;
        // modify :
        double shooterVelocity = -1;


//        if(result != null && result.isValid()){
//            tx = result.getTx();
//            ty = result.getTy();
//            botpose = result.getBotpose();
//
//            // --- Distance Calculation ---
//            double angleToGoalDegrees = LIMELIGHT_MOUNT_ANGLE_DEGREES + ty;
//            double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
//            distanceInches = (APRILTAG_HEIGHT - LIMELIGHT_LENS_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);
//            // --- Velocity Calculation ---
//
//            // lights, camera, action!!
//            if(distanceInches < 80){
//                // lights change color to green indicating that correct distance is shown
//                blinkin.setPosition(0.25);
//                // Pause for 2.5 second to alert the drivers
//                try {
//                    sleep(2500);
//                } catch (InterruptedException e) {
//                    throw new RuntimeException(e);
//                }
//            }
//            else if (time==90) {
//                blinkin.setPosition(-0.97);
//                // Show rainbow pattern for 1.5 seconds to alert driver that 1.5 minutes are left
//                try {
//                    sleep(1500);
//                } catch (InterruptedException e) {
//                    throw new RuntimeException(e);
//                }
//            }
//            else if (time==10) {
//                blinkin.setPosition(-0.97);
//                // Show rainbow pattern for 0.8 seconds to alert driver that 10 seconds are left
//                try {
//                    sleep(800);
//                } catch (InterruptedException e) {
//                    throw new RuntimeException(e);
//                }
//            }
//            else{
//                blinkin.setPosition(0.61);
//                telemetry.addLine("No Target Found");
//            }
//            if(gamepad2.b){
//                if (Pattern.equals("ID:21; GPP")) {
//                    // Initially check for green
//                    if (((normRed + normBlue)/ tolerance_green) < normGreen) {
//                        telemetry.addLine("Green");
//                        detectingPurple = false;
//                        detectingBlue = false;
//                        // shoot the ball if green
//                        gate.setPosition(90);
//                        // HERE HERE HERE HERE HERE HERE HERE HERE HERE HERE HERE HERE HERE HERE HERE
//                    }
//                    else{
//                        windmill.setPower(0.3);
//                        try {
//                            sleep(100);
//                        } catch (InterruptedException e) {
//                            throw new RuntimeException(e);
//                        }
//                        windmill.setPower(0);
//                    }
//                }
//                if (Pattern.equals("ID:22; PGP")) {
//                    // launches accordingly
//                }
//                if (Pattern.equals("ID:23; PPG")) {
//                    // launches accordingly
//                }
//            }
//        }
////        if (result == null || !result.isValid()) {
////            // Show red even when april tag is not visible to bot
////            blinkin.setPosition(0.61);
////            telemetry.addLine("No Target Found");
////        }
//
//
//            // Driver sets the pattern w/ Dpad
//        if(gamepad1.dpad_left){
//            Pattern = "ID:21; GPP";
//            telemetry.addLine(Pattern);
//        }
//        if(gamepad1.dpad_up){
//            Pattern = "ID:22; PGP";
//            telemetry.addLine(Pattern);
//
//        }
//        if(gamepad1.dpad_right){
//            Pattern = "ID:23; PPG";
//            telemetry.addLine(Pattern);
//
//        }
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

        // Emergency Stop w/ PS
        if (gamepad1.psWasPressed() || gamepad2.psWasPressed()){
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            intake.setPower(0);
            shooter.setPower(0);
            windmill.setPower(0);
            gate.setPosition(0);
        }


        // Intake w/ Bumpers
        if (gamepad2.left_bumper){
            intake.setPower(1);
        }
        if(gamepad2.right_bumper){
            intake.setPower(-1);
        }
        if(gamepad2.left_bumper && gamepad2.right_bumper){
            intake.setPower(0);
        }
        if(gamepad2.a){
            shooter.setPower(1);
        }



        telemetry.setMsTransmissionInterval(30);

        telemetry.update();
        update();

    }
}


