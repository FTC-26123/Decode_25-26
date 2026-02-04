package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.math.RoundingMode;
import java.text.DecimalFormat;

@TeleOp(name = "newRobotTesting")
public class newRobotTesting extends OpMode {

    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    public DcMotorEx launcherRight;
    public DcMotor index;
    public DcMotor intake;
    public DcMotorEx launcherLeft;
    public Servo light;

    public CRServo gate;
    long dualVelocity = 2250;
    boolean gamepadBWasPressed = false;

    boolean shooterIsBusy = false;
    final double motorStallThreshold = 8; // amps
    boolean shooterIsStalled = false;
    boolean indexRunByInput = false;

    public int menuPosition = 1;
    public String menuType = "main";

    public ElapsedTime lightTimer = new ElapsedTime();

    public ElapsedTime menuTimer = new ElapsedTime();

    @Override
    public void init() {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        launcherRight = hardwareMap.get(DcMotorEx.class, "launcher");
        index = hardwareMap.get(DcMotor.class, "windmill");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcherLeft = hardwareMap.get(DcMotorEx.class, "launcherLeft");

        light = hardwareMap.get(Servo.class, "light");

        gate = hardwareMap.get(CRServo.class, "gate");

        launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);;
        launcherLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        index.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

        if (gamepad2.a && !gamepad1.start && !gamepad2.start) {
            index.setPower(1);
            gate.setPower(1);
            indexRunByInput = true;
        } else if (gamepad2.b && !gamepad1.start && !gamepad2.start) {
            index.setPower(-1);
            intake.setPower(-1);
            gate.setPower(-1);
            indexRunByInput = true;
            gamepadBWasPressed = true;
        } else if (!gamepad2.a && !gamepad2.b) {
            index.setPower(0);
            gate.setPower(0);
        } if (!gamepad2.b && gamepadBWasPressed) {
            intake.setPower(0);
            gamepadBWasPressed = false;
        }

        DecimalFormat decimalFormat = new DecimalFormat("#,###");
        decimalFormat.setRoundingMode(RoundingMode.HALF_UP);

        telemetry.addData("Target Launcher Velocity", dualVelocity);
        telemetry.addLine("Launcher Left Velocity: " + launcherLeft.getVelocity() + " ... Launcher Left RPM: " + decimalFormat.format(launcherLeft.getVelocity() / 28 * 60) + "RPM");
        telemetry.addLine("Launcher Right Velocity: " + launcherRight.getVelocity() + " ... Launcher Right RPM: " + decimalFormat.format(launcherLeft.getVelocity() / 28 * 60) + "RPM");
        if (gamepad2.left_trigger > 0.4) {
            launcherLeft.setVelocity(dualVelocity);
            launcherRight.setVelocity(dualVelocity);
            shooterIsBusy = true;
        } if (gamepad2.left_bumper) {
            launcherLeft.setVelocity(0);
            launcherRight.setVelocity(0);
            shooterIsBusy = false;
        } if (shooterIsBusy && launcherRight.getVelocity() > (dualVelocity - 45) && launcherLeft.getVelocity() > (dualVelocity - 45)) {
            light.setPosition(0.47);
        } else if (shooterIsBusy && launcherRight.getVelocity() < (dualVelocity - 45) && launcherLeft.getVelocity() < (dualVelocity - 45)) {
            light.setPosition(0.29);
        } else if (!shooterIsBusy) {
            light.setPosition(0);
        }
        if (shooterIsBusy && Math.abs(launcherLeft.getVelocity() - launcherRight.getVelocity()) > 50) {
            lightTimer.reset();
            if (lightTimer.milliseconds() > 300 && lightTimer.milliseconds() < 599) {
                light.setPosition(0.6);
            } if (lightTimer.milliseconds() > 600) {
                light.setPosition(0.8);
                lightTimer.reset();
            }
        }

        if (shooterIsStalled) {
            indexRunByInput = false;
            index.setPower(-1);
            gate.setPower(-1);
        } if (!shooterIsStalled && !indexRunByInput) {
            index.setPower(0);
            gate.setPower(1);
        }

        telemetry.addData("Left Launcher Amps", launcherLeft.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Right Launcher Amps", launcherRight.getCurrent(CurrentUnit.AMPS));

        if (shooterIsBusy) { // checking for stall motor
            if (launcherLeft.getCurrent(CurrentUnit.AMPS) > motorStallThreshold) {
                telemetry.addLine("Left Launcher Stalled!");
                shooterIsStalled = true;
            } if (launcherRight.getCurrent(CurrentUnit.AMPS) > motorStallThreshold) {
                telemetry.addLine("Right Launcher Stalled!");
                shooterIsStalled = true;
            } if (launcherLeft.getCurrent(CurrentUnit.AMPS) < motorStallThreshold) {
                shooterIsStalled = false;
            } if (launcherRight.getCurrent(CurrentUnit.AMPS) < motorStallThreshold) {
                shooterIsStalled = false;
            }
        }

        if (shooterIsStalled) {
            indexRunByInput = false;
            index.setPower(-1);
        } if (!shooterIsStalled && !indexRunByInput) {
            index.setPower(0);
        }

        telemetry.addLine(" ");
        telemetry.addLine("dpad to navigate, start to select");
        telemetry.addLine(" ");

        if (menuType.equals("main")) {
            if (menuPosition == 1) {
                telemetry.addLine("[>] Preset Velocity -> 950");
                telemetry.addLine("[] Preset Velocity -> 1000");
                telemetry.addLine("[] Preset Velocity -> 1200");
                telemetry.addLine("[] Custom Velocity");
                if (gamepad2.start) {
                    dualVelocity = 950;
                }
            } else if (menuPosition == 2) {
                telemetry.addLine("[] Preset Velocity -> 950");
                telemetry.addLine("[>] Preset Velocity -> 1000");
                telemetry.addLine("[] Preset Velocity -> 1200");
                telemetry.addLine("[] Custom Velocity");
                if (gamepad2.start) {
                    dualVelocity = 1000;
                }
            } else if (menuPosition == 3) {
                telemetry.addLine("[] Preset Velocity -> 950");
                telemetry.addLine("[] Preset Velocity -> 1000");
                telemetry.addLine("[>] Preset Velocity -> 1200");
                telemetry.addLine("[] Custom Velocity");
                if (gamepad2.start) {
                    dualVelocity = 1200;
                }
            } else if (menuPosition == 4) {
                telemetry.addLine("[] Preset Velocity -> 950");
                telemetry.addLine("[] Preset Velocity -> 1000");
                telemetry.addLine("[] Preset Velocity -> 1200");
                telemetry.addLine("[>] Custom Velocity");
                if (gamepad2.start) {
                    menuType = "customVel";
                }
            } else { telemetry.addLine("error"); }

            if (gamepad2.dpad_up && menuPosition != 1 && menuTimer.milliseconds() > 75) {
                menuPosition--;
                menuTimer.reset();
            } if (gamepad2.dpad_down && menuPosition != 4 && menuTimer.milliseconds() > 75) {
                menuPosition++;
                menuTimer.reset();
            }
        } else if (menuType.equals("customVel")) {
            telemetry.addLine("Gamepad2.dpad_up -> increase velocity by 10");
            if (gamepad2.dpad_up) {
                dualVelocity += 10;
            }

            telemetry.addLine("Gamepad2.dpad_down -> decrease velocity by 10");
            if (gamepad2.dpad_down) {
                dualVelocity -= 10;
            }

            telemetry.addLine("[>] Exit");
            if (gamepad2.start) {
                menuType = "main";
            }
        }

        telemetry.update();

    }
}
