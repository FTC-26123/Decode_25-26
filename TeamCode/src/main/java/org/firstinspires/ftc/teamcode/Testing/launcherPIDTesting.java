package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "newRobotTesting_mainFollower")
public class launcherPIDTesting extends OpMode {

    public DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    public DcMotorEx launcherLeft, launcherRight;
    public DcMotor index, intake;

    private double targetRPM = 3000; // starting target
    private final double TICKS_PER_REV = 28.0; // RS-555 encoder
    private ElapsedTime pidTimer = new ElapsedTime();
    private double lastTime = 0;

    private PID leftPID, rightPID;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        launcherLeft = hardwareMap.get(DcMotorEx.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotorEx.class, "launcher");
        index = hardwareMap.get(DcMotor.class, "windmill");
        intake = hardwareMap.get(DcMotor.class, "intake");

        launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);;
        launcherLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        index.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // PID tuned for high-speed RS-555 motors
        leftPID = new PID(0.0005, 0.00001, 0.00005);   // Master PID
        rightPID = new PID(0.0005, 0.00001, 0.00005);  // Slave PID

        pidTimer.reset();
        lastTime = pidTimer.seconds();
    }

    @Override
    public void loop() {
        double now = pidTimer.seconds();
        double dt = now - lastTime;
        lastTime = now;

        // Master motor velocity (ticks/sec)
        double leftVelocityTicks = launcherLeft.getVelocity();
        double leftRPM = (leftVelocityTicks / TICKS_PER_REV) * 60.0;

        // Master PID: left motor tries to reach targetRPM
        double leftPower = leftPID.update(targetRPM, leftRPM, dt);
        leftPower = clamp(leftPower, -1, 1);
        if (gamepad2.y) launcherLeft.setPower(leftPower);

        // Slave motor velocity (ticks/sec)
        double rightVelocityTicks = launcherRight.getVelocity();
        double rightRPM = (rightVelocityTicks / TICKS_PER_REV) * 60.0;

        // Slave PID: right motor tries to match **left motor's actual RPM**
        double rightPower = rightPID.update(leftRPM, rightRPM, dt);
        rightPower = clamp(rightPower, -1, 1);
        if (gamepad2.x) launcherRight.setPower(rightPower);

        // Intake / Index controls
        if (gamepad2.a) intake.setPower(1); else intake.setPower(0);
        if (gamepad2.b) index.setPower(1); else index.setPower(0);

        // Stop all
        if (gamepad2.ps) {
            launcherLeft.setPower(0);
            launcherRight.setPower(0);
            intake.setPower(0);
            index.setPower(0);
        }

        // Reverse / forward intake & index
        if (gamepad2.start) {
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            index.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        if (gamepad2.back) {
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            index.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        // Adjust target RPM
        if (gamepad2.dpad_up) targetRPM += 50;
        if (gamepad2.dpad_down) targetRPM -= 50;

        // Telemetry
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Left RPM", leftRPM);
        telemetry.addData("Right RPM", rightRPM);
        telemetry.addData("Left Power", leftPower);
        telemetry.addData("Right Power", rightPower);
        telemetry.update();
    }

    // Helper clamp
    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    // PID class
    static class PID {
        double kP, kI, kD;
        double integral = 0;
        double lastError = 0;

        PID(double p, double i, double d) {
            kP = p;
            kI = i;
            kD = d;
        }

        double update(double target, double current, double dt) {
            double error = target - current;
            integral += error * dt;
            double derivative = (error - lastError) / dt;
            lastError = error;
            return kP * error + kI * integral + kD * derivative;
        }
    }
}