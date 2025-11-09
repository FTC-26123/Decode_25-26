package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.odometry.GoBildaPinpointDriver;

import java.util.function.BooleanSupplier;

@DeviceProperties(
        name = "goBILDA® Pinpoint Odometry Computer",
        xmlTag = "goBILDAPinpoint",
        description ="goBILDA® Pinpoint Odometry Computer (IMU Sensor Fusion for 2 Wheel Odometry)"
)
public class Commons {
    public static final float MOTOR_MULTIPLIER_PERCENTAGE_CAP = 0.5F;
    public static float AUTON_MOTOR_MULTIPLIER_PERCENTAGE_CAP = 0.8f;
    public static final float ARMROT_SPEED_CAP = 0.7F;

    public static final double TPI_ODO = 505.325937f;
    public static final double TPI_MOTOR = 41.8f;

    public static DcMotor frontLeftMotor;
    public static DcMotor frontRightMotor;
    public static DcMotor backLeftMotor;
    public static DcMotor backRightMotor;

    public static DcMotor intake;

    public static DcMotor windmill;
    public static Servo gate;

    public int apriltag_height = 25;

    public static DcMotorEx shooter;

    public static Limelight3A limelight;

    public static IMU imu;

    public static GoBildaPinpointDriver odo;

    public static boolean initialized = false;

    public static BooleanSupplier opModeIsActive;

    public static Telemetry telemetry;

    public static NormalizedColorSensor colorSensor;

    private final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 135;
    private final double LIMELIGHT_LENS_HEIGHT_INCHES = 15;
    private final double GOAL_HEIGHT_INCHES = 40;
    private final double APRILTAG_HEIGHT = 25;

    public static boolean isBusy = false;

    public static void init(HardwareMap hardwareMap, BooleanSupplier opModeIsActive, Telemetry telemetry) {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        intake = hardwareMap.get(DcMotor.class, "intake");
        windmill = hardwareMap.get(DcMotor.class, "windmill");
        gate = hardwareMap.get(Servo.class, "gate");
        shooter = hardwareMap.get(DcMotorEx.class, "launcher");


        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "revColorV3");
        colorSensor.setGain(7);


        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);





        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT))
        );

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(101.6, -165.1);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

        Commons.opModeIsActive = opModeIsActive;
        Commons.telemetry = telemetry;

        initialized = true;


    }

    public static int initWarning() {
        if (!initialized) {
            System.err.println("Initialize commons first! - \"Commons.init();\"");
            return 1;
        }
        return 0;
    }
/** <p>
 * @param targetTurnAngle This function turns <strong>left</strong> in the respective turn angle received.
 * @param speed This function denotes the speed of the turn.
 *              This function uses PID to calculate the turn more accurately.
 *             </p>
 */

    public static void PID_rotateLeft(double targetTurnAngle, double speed) throws InterruptedException {
        if (initWarning()==1) {return;}

        isBusy = true;

        imu.resetYaw();

        double currentAngle = getYawAngle(); // The current angle
        double targetAngle; // The final angle we want to be at when the turn is finished

        targetAngle = targetTurnAngle;

        if (targetAngle < 0.05 && targetAngle > -0.05) {
            return;
        }

        double error = targetAngle - currentAngle; // How much we have to turn to get to targetAngle on the IMU YAW (degrees; right is positive)
        double originalError = targetAngle - currentAngle;

        double P;
        double I = 0;
        double D;

        double Kp = Commons.AUTON_MOTOR_MULTIPLIER_PERCENTAGE_CAP;
        double Ki = 0.015;
        double Kd = -0.005;

        double maxI = 0.6;

        while ((error <= -2 || error >= 2) && opModeIsActive.getAsBoolean()) {

//            print("Angle", Double.toString(Commons.getYawAngle()));
//            print("Error", Double.toString(error));

//            Checks
            if (Math.abs(I) > maxI) {
                I = maxI - error/originalError * Ki;
            }
//            PID Calculations
            P = error/originalError * Kp;
            I += error/originalError * Ki;
            D = error/originalError * Kd;
//            Powering Motors
            Commons.startRotateLeft((P+I+D)*speed);
//            Update Variables
            error = targetAngle - getYawAngle();
//            Sleep to prevent CPU stress
            Thread.sleep(5);

        }

        Commons.stopMotorsRotateLeft();
        Commons.stopMotors();

        isBusy = false;
    }
    /** <p>
     * @param targetTurnAngle This function turns <strong>right</strong> in the respective turn angle received.
     * @param speed This function denotes the speed of the turn.
     *              This function uses PID to calculate the turn more accurately.
     *             </p>
     */
    public static void PID_rotateRight(double targetTurnAngle, double speed) throws InterruptedException {
        if (initWarning()==1) {return;}

        isBusy = true;

        imu.resetYaw();

        double currentAngle = getYawAngle(); // The current angle
        double targetAngle; // The final angle we want to be at when the turn is finished

        targetAngle = -targetTurnAngle;

        if (targetAngle < 0.05 && targetAngle > -0.05) {
            return;
        }

        double error = targetAngle - currentAngle; // How much we have to turn to get to targetAngle on the IMU YAW (degrees; right is positive)
        double originalError = targetAngle - currentAngle;

        double P;
        double I = 0;
        double D;

        double Kp = Commons.AUTON_MOTOR_MULTIPLIER_PERCENTAGE_CAP;
        double Ki = 0.015;
        double Kd = -0.005;

        double maxI = 0.6;

        while ((error <= -2 || error >= 2) && opModeIsActive.getAsBoolean()) {

//            print("Angle", Double.toString(Commons.getYawAngle()));
//            print("Error", Double.toString(error));

//            Checks
            if (Math.abs(I) > maxI) {
                I = maxI - error/originalError * Ki;
            }
//            PID Calculations
            P = error/originalError * Kp;
            I += error/originalError * Ki;
            D = error/originalError * Kd;
//            Powering Motors
            Commons.startRotateRight((P+I+D)*speed);
//            Update Variables
            error = targetAngle - getYawAngle();
//            Sleep to prevent CPU stress
            Thread.sleep(5);

        }

        Commons.stopMotorsRotateRight();
        Commons.stopMotors();

        isBusy = false;
    }

    /**Moves forward precisely using PID and odometry.<br><br>
     @param targetInches The number of inches to move forward relative to the robot position<br>
     @param maxSpeed The maximum speed the robot at go to reach the target position. This value is
     multiplied by 0.8. To counter this, multiply speed by 1.25 for more accurate speed. The speed
     will scale from the provided maximum to I+((error/originalError)*(timesLooped-1)), I being
     the integral value in PID at the very end*/
    public static void PID_forward(double targetInches, double maxSpeed) throws InterruptedException {
        if (initWarning() == 1) return;

        isBusy = true;

        odo.update();
        imu.resetYaw();

        double startPosition = getXPosition();
        double targetPosition = startPosition + targetInches;

        // PID constants (start with these, tune later)
        double Kp = 0.04;      // proportional
        double Ki = 0;         // integral
        double Kd = 0.01;      // derivative
        double aKp = 0.02;     // angle correction

        double previousError = 0;
        double integral = 0;

        double error = targetPosition - getXPosition();

        while (Math.abs(error) > 0.5 && opModeIsActive.getAsBoolean()) { // tolerance 0.5 inch
            odo.update();

            double currentPosition = getXPosition();
            error = targetPosition - currentPosition;

            // PID calculations
            double P = Kp * error;
            integral += Ki * error;
            double D = Kd * (error - previousError);
            previousError = error;

            double output = P + integral + D;
            output = Math.max(-maxSpeed, Math.min(maxSpeed, output)); // clamp power

            // angle correction
            double angleError = getYawAngle();
            double angleAdjust = aKp * angleError;

            // set mecanum motor powers
            frontLeftMotor.setPower(output + angleAdjust);
            frontRightMotor.setPower(output - angleAdjust);
            backLeftMotor.setPower(output + angleAdjust);
            backRightMotor.setPower(output - angleAdjust);

            // live telemetry
            telemetry.addData("X Position", getXPosition());
            telemetry.addData("Error", error);
            telemetry.update();
        }

        // stop motors
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        isBusy = false;
    }


    /**Moves backward precisely using PID and odometry.<br><br>
     @param targetInches The number of inches to move backward relative to the robot position<br>
     @param speed The maximum speed the robot at go to reach the target position. This value is
     multiplied by 0.8. To counter this, multiply speed by 1.25 for more accurate speed. The speed
     will scale from the provided maximum to I+((error/originalError)*(timesLooped-1)), I being
     the integral value in PID at the very end*/
    public static void PID_backward(int targetInches, double speed) throws InterruptedException {
        if (initWarning()==1) {return;}

        isBusy = true;

//        odo.setPosition(new Pose2D(DistanceUnit.MM, odo.getPosition().getX(DistanceUnit.MM), odo.getPosition().getY(DistanceUnit.MM), AngleUnit.RADIANS, 0));
        odo.update();
        imu.resetYaw();

        double currentPosition = getXPosition();
        double targetPosition = currentPosition - (targetInches);

        double error = targetPosition - currentPosition;
        double originalError = targetPosition - currentPosition;

        double errorAngle = getYawAngle();

        double P;
        double I = 0.1;
        double D;

        double aP;

        double Kp = Commons.AUTON_MOTOR_MULTIPLIER_PERCENTAGE_CAP;
        double Ki = 0.02f;
        double Kd = 0.08f;

        double aKp = 0.05;
        double maxI = 0.5f;

        while ((error <= -1 || error >= 1) && opModeIsActive.getAsBoolean()) { // As long as robot isn't within an inch of target pos, loop
            odo.update();
//            Checks
            if (Math.abs(I) > maxI) {
                I = maxI - error/originalError * Ki;
            }
//            PID Calculations
            P = error/originalError * Kp; // Distance from target / original distance from target
            I += error/originalError * Ki;
            D = error/originalError * Kd;
            aP = errorAngle * aKp; // Angle / Original Angle
//            Powering Motors
            double PID = (P+I+D)*speed;
            if (PID > speed) {
                PID = speed;
            } else if (Math.abs(PID) > speed) {
                PID = -speed;
            }

            double FaP = (aP)*speed;

            frontLeftMotor.setPower(-PID + FaP);
            frontRightMotor.setPower(-PID - FaP);
            backLeftMotor.setPower(-PID + FaP);
            backRightMotor.setPower(-PID - FaP);

//            Updating Variables
            error = targetPosition - getXPosition();
            errorAngle = getYawAngle();
        }

        Commons.stopMotorsBackward();
        Commons.stopMotors();

        isBusy = false;
    }

    public static void PID_goto(double targetInchesX, double targetInchesY, boolean forwardFirst, double speed) throws InterruptedException {
        if (initWarning()==1) {return;}

        isBusy = true;

        odo.update();

        double currentPositionX = getXPosition();
        double targetPositionX = targetInchesX + currentPositionX;

        double currentPositionY = getYPosition();
        double targetPositionY = targetInchesY + currentPositionY;

        double errorX = targetPositionX - currentPositionX;
        double originalErrorX = targetPositionX - currentPositionX;

        double errorY = targetPositionY - currentPositionY;
        double originalErrorY = targetPositionY - currentPositionY;

//        telemetry.addData("Target Pos: ", targetPosition + " ");
//        telemetry.addData("Current Pos: ", currentPosition + " ");
//        telemetry.addData("Error: ", error + " ");
//        odo.update();
//        telemetry.addData("X: ", odo.getPosition().getX(DistanceUnit.INCH) + " ");
//        telemetry.update();
//        sleep(5000);

        double Px;
        double Ix = 0.1;
        double Dx;

        double Py;
        double Iy = 0.1;
        double Dy;

        double Kp = Commons.AUTON_MOTOR_MULTIPLIER_PERCENTAGE_CAP;
        double Ki = 0.02f;
        double Kd = 0.08f;

        double maxI = 0.5f;

        telemetry.addData("targetInchesY: ", targetInchesY);
        telemetry.addData("errorY: ", errorY);
        telemetry.addData("originalErrorY*Kp: ", originalErrorY*Kp);
        telemetry.update();
        sleep(6000);

        while (((errorX <= -1 || errorX >= 1)||(errorY <= -1 || errorY >= 1)) && opModeIsActive.getAsBoolean()) { // As long as robot isn't within an inch of target pos, loop
            odo.update();
//            Checks
            if (Math.abs(Ix) > maxI) {
                Ix = maxI - errorX/Math.abs(originalErrorX * Ki);
            } if (Math.abs(Iy) > maxI) {
                Iy = maxI - errorY/Math.abs(originalErrorY * Ki);
            }
//            PID Calculations
            if (forwardFirst) {
                Px = errorX/Math.abs(originalErrorX * Kp); // Distance from target / original distance from target
            } else {
                Px = (targetInchesX - errorX)/Math.abs(originalErrorX * Kp);
            }
            Ix += errorX/Math.abs(originalErrorX) * Ki;
            Dx = errorX/Math.abs(originalErrorX) * Kd;

            if (forwardFirst) {
                Py = (targetInchesY - errorY)/Math.abs(originalErrorY * Kp);
            } else {
                Py = errorY/Math.abs(originalErrorY * Kp);
            }
            Iy += errorY/Math.abs(originalErrorY * Ki);
            Dy = errorY/Math.abs(originalErrorY * Kd);

//            Powering Motors (Forward & Y Error)
            double PIDx = (Px+Ix+Dx)*speed;
            double PIDy = (Py)*speed;

            telemetry.addData("errorY: ", errorY);
            telemetry.addData("Py: ", Py*speed);
            telemetry.update();

            frontLeftMotor.setPower((PIDx - PIDy)*0.5);
            frontRightMotor.setPower((PIDx + PIDy)*0.5);
            backLeftMotor.setPower((PIDx + PIDy)*0.5);
            backRightMotor.setPower((PIDx - PIDy)*0.5);

//            Updating Variables
            errorX = targetPositionX - getXPosition();
            errorY = targetPositionY - getYPosition();
//            telemetry.addData("ErrorY: ", Double.toString(errorY));
//            telemetry.update();
        }


        Commons.stopMotors();

        isBusy = false;
    }

    //    Robot Values Getters
    public static double getYawAngle() {
        if (initWarning()==1) {return 181;}

        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    @Deprecated
    public static int getMotorPosition(int motor) {
        if (initWarning()==1) {return 181;}

        switch (motor) {
            case 0:
                return frontLeftMotor.getCurrentPosition();
            case 1:
                return frontRightMotor.getCurrentPosition();
            case 2:
                return backLeftMotor.getCurrentPosition();
            case 3:
                return backRightMotor.getCurrentPosition();
            default:
                System.err.println("\"" + Integer.toString(motor) + "\" is not recognized as a motor; only values 0 - 5 are supported.");
        }

        return 0;
    }

    //    Basic Robot Movement

    public static void moveForward(int inches, double speed) throws InterruptedException {
        if (initWarning()==1) {return;}

        isBusy = true;

        double targetPosition = (inches) + odo.getPosition().getX(DistanceUnit.INCH);

//        if (odo.getPosition().getX(DistanceUnit.INCH) < targetPosition) {
        while (odo.getPosition().getX(DistanceUnit.INCH) < targetPosition && opModeIsActive.getAsBoolean()) {
            odo.update();
            startForward(speed*AUTON_MOTOR_MULTIPLIER_PERCENTAGE_CAP);
            telemetry.update();
            telemetry.addData("X", Math.ceil(Commons.odo.getPosition().getX(DistanceUnit.INCH)));
            telemetry.addData("Y",  Math.ceil(Commons.odo.getPosition().getY(DistanceUnit.INCH)));
            telemetry.update();
        }
//        } else if (odo.getPosition().getX(DistanceUnit.INCH) > targetPosition) {
//            while (odo.getPosition().getX(DistanceUnit.INCH) > targetPosition && opModeIsActive.getAsBoolean()) {
//                odo.update();
//                startForward(speed);
//                telemetry.addData("X", Math.ceil(Commons.odo.getPosition().getX(DistanceUnit.INCH)));
//                telemetry.addData("Y",  Math.ceil(Commons.odo.getPosition().getY(DistanceUnit.INCH)));
//                telemetry.update();
//            }
//        }

        Commons.stopMotorsForward();
        stopMotors();

        isBusy = false;
    }


    public static void moveBackward(int inches, double speed) throws InterruptedException {
        if (initWarning()==1) {return;}

        isBusy = true;

        int targetPosition = (int)(inches * TPI_MOTOR) + frontRightMotor.getCurrentPosition();

        if (frontRightMotor.getCurrentPosition() > targetPosition) {
            while (frontRightMotor.getCurrentPosition() > targetPosition && opModeIsActive.getAsBoolean()) {
                startBackward(-speed*AUTON_MOTOR_MULTIPLIER_PERCENTAGE_CAP);
                telemetry.addData("current pos: ", Integer.toString(backRightMotor.getCurrentPosition()));
                telemetry.update();
            }
        } else if (frontRightMotor.getCurrentPosition() < targetPosition) {
            while (frontRightMotor.getCurrentPosition() < targetPosition && opModeIsActive.getAsBoolean()) {
                startBackward(speed);
            }
        }

        Commons.stopMotorsBackward();
        stopMotors();

        isBusy = false;
    }

    public static void startForward(double speed) {
        if (initWarning()==1) {return;}

        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);
    }

    public static void startBackward(double speed) {
        if (initWarning()==1) {return;}

        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(-speed);
        backLeftMotor.setPower(-speed);
        backRightMotor.setPower(-speed);
    }

    public static void startLateralLeft(double speed) {
        if (initWarning()==1) {return;}

        frontLeftMotor.setPower(-speed);
        frontRightMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(-speed);
    }

    public static void startLateralRight(double speed) {
        if (initWarning()==1) {return;}

        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(-speed);
        backLeftMotor.setPower(-speed);
        backRightMotor.setPower(speed);
    }

    public static void startRotateRight(double speed) {
        if (initWarning()==1) {return;}

        frontLeftMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        frontRightMotor.setPower(-speed);
        backRightMotor.setPower(-speed);
    }

    public static void startRotateLeft(double speed) {
        if (initWarning()==1) {return;}

        frontLeftMotor.setPower(-speed);
        backLeftMotor.setPower(-speed);
        frontRightMotor.setPower(speed);
        backRightMotor.setPower(speed);
    }

    public static void lateralLeft(double inches, double speed) throws InterruptedException {
        if (initWarning()==1) {return;}

        isBusy = true;
        odo.update();

        double targetPosition = (inches) + getYPosition();
        while ((getYPosition() < targetPosition) && opModeIsActive.getAsBoolean()) {
            odo.update();
            startLateralLeft(speed*AUTON_MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        }
        Commons.stopMotorsLateralLeft();
        stopMotors();

        isBusy = false;
    }

    public static void lateralRight(double inches, double speed) throws InterruptedException {
        if (initWarning()==1) {return;}

        isBusy = true;
        odo.update();

        double targetPosition = (-inches) + getYPosition();
        while (getYPosition() > targetPosition && opModeIsActive.getAsBoolean()) {
            odo.update();
            startLateralRight(speed*AUTON_MOTOR_MULTIPLIER_PERCENTAGE_CAP);
        }
        Commons.stopMotorsLateralRight();
        stopMotors();

        isBusy = false;
    }

    public static void stopMotors() {
        if (initWarning()==1) {return;}

        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    public static void print(String value, String value1) {
        if (initWarning()==1) {return;}

        telemetry.addData(value, value1);
        telemetry.update();
    }

    @Deprecated
    public static void setMotorMode(DcMotor.RunMode runMode) {
        if (initWarning()==1) {return;}

        frontLeftMotor.setMode(runMode);
        frontRightMotor.setMode(runMode);
        backLeftMotor.setMode(runMode);
        backRightMotor.setMode(runMode);
    }

    @Deprecated
    public static void setMotorTargetPosition(int targetPosition) {
        if (initWarning()==1) {return;}

        frontLeftMotor.setTargetPosition(targetPosition);
        frontRightMotor.setTargetPosition(targetPosition);
        backLeftMotor.setTargetPosition(targetPosition);
        backRightMotor.setTargetPosition(targetPosition);
    }

    public static void stopMotorsForward() throws InterruptedException {
        if (initWarning()==1) {return;}

        startForward(-0.3);
        sleep(50);
        stopMotors();
    }

    public static void stopMotorsBackward() throws InterruptedException {
        if (initWarning()==1) {return;}

        startBackward(-0.3);
        sleep(50);
        stopMotors();
    }

    public static void stopMotorsRotateLeft() throws InterruptedException {
        if (initWarning()==1) {return;}

        startRotateLeft(-0.2);
        sleep(50);
        stopMotors();
    }

    public static void stopMotorsRotateRight() throws InterruptedException {
        if (initWarning()==1) {return;}

        startRotateRight(-0.2);
        sleep(50);
        stopMotors();
    }

    public static void stopMotorsLateralLeft() throws InterruptedException {
        if (initWarning()==1) {return;}

        startLateralLeft(-0.3);
        sleep(50);
        stopMotors();
    }

    public static void stopMotorsLateralRight() throws InterruptedException {
        if (initWarning()==1) {return;}

        startLateralRight(-0.3);
        sleep(50);
        stopMotors();
    }


    public static void initializeColorSensor(boolean displayColorValues, boolean displayDetectedColor) throws InterruptedException {

        /* detectedColor= color that is being detected - use for code
         * displayColorValues = true ... displays color values int col and double hue
         * displayDetectedColor = true ... displays which color is being detected (eg. red)   */

        // 2 is green
        // 4 is purple

        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        int col = colors.toColor();
        double hue = JavaUtil.colorToHue(col);

        byte detectedColor;

        if (displayColorValues) {
            telemetry.addData("Detected Hue", hue);
            telemetry.addData("Detected Color", col);
        }

        if (hue > 15 && hue < 60) {
            if (displayDetectedColor) {
                telemetry.addLine("Detected Color: Red");
            }
            detectedColor = 1;

        } else if (hue > 90 && hue < 180) {
            if (displayDetectedColor) {
                telemetry.addLine("Detected Color: Green");
            }
            detectedColor = 2;
        } else if (hue > 200 && hue < 210) {
            if (displayDetectedColor) {
                telemetry.addLine("Detected Color: Blue");
            }
            detectedColor = 3;

        } else if (hue > 225 && hue < 350) {
            if (displayDetectedColor) {
                telemetry.addLine("Detected Color: Purple");
            }
            detectedColor = 4;

        }

        telemetry.update();


    }
/** <p> Reports in inches</p>*/
    public static double getXPosition() {
        return odo.getEncoderX() / (19.89436789f * 25.4);
    }
    /** <p> Reports in inches</p>*/
    public static double getYPosition() {
        return odo.getEncoderY() / (19.89436789f * 25.4);
    }

    /** <p>
     * @param velocity The velocity that the shooter runs at (a value from <strong>1</strong> to <strong>~2100</strong>) </p><p>This code automatically sets the direction to reverse.</p> */
    public static void runShooter(double velocity) {
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setVelocity(velocity);
    }




}