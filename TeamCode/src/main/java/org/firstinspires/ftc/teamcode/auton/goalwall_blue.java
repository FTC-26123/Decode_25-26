package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import static org.firstinspires.ftc.teamcode.Commons.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.Commons;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous(name = "GoalWall_BLUE_TIME")
public class goalwall_blue extends LinearOpMode {

    int apriltag_height = 25;
    final double limelightMountAngleDegrees = 35;
    final double limelightLensHeightInches = 14;
    double ty = 0;
    double distance = 0;
    double flywheelRadius = 1.89; // inches

    int motif_pattern = 21;

    final double shooterdump = 0.05;

    final double neutral = 0.5;

    final double dumping = 0.8;

    // --- LIMELIGHT + SORTER ---
    public double getdistance(double y) throws InterruptedException {
        double angleToGoalDegrees = limelightMountAngleDegrees + ty;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
        distance = (apriltag_height - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
        return distance;
    }

    public void sorter(int pattern, double distance) throws InterruptedException {
        final int goalHeight = 40;
        double launchAngleDegrees = 50.0;
        double launchHeight = 10.0;
        double fudgeFactor = 0.3;
        double g = 386.0; // in/s^2
        double theta = Math.toRadians(launchAngleDegrees);
        double y = goalHeight - launchHeight;

        double denominator = 2 * Math.pow(Math.cos(theta), 2) * (distance * Math.tan(theta) - y);
        if (denominator <= 0)
            telemetry.addLine("Too close");
        else {
            double velocity = Math.sqrt((g * distance * distance) / denominator) * (1 + fudgeFactor);

            int detectedColor = 0;
            ArrayList<Integer> color_pattern = new ArrayList<>();

            switch (pattern) {
                case 21:
                    color_pattern.addAll(Arrays.asList(1, 2, 2));
                    break;
                case 22:
                    color_pattern.addAll(Arrays.asList(2, 1, 2));
                    break;
                case 23:
                    color_pattern.addAll(Arrays.asList(2, 2, 1));
                    break;
            }

            while (!color_pattern.isEmpty()) {
                windmill.setPower(0.5);
                sleep(1000);
                NormalizedRGBA colors = colorSensor.getNormalizedColors();
                int col = colors.toColor();
                double hue = JavaUtil.colorToHue(col);
                if (hue > 90 && hue < 180) detectedColor = 1; // green
                else if (hue > 225 && hue < 350) detectedColor = 2; // purple

                double angularVelocityRad = velocity / flywheelRadius;

                if (detectedColor == color_pattern.get(0)) {
                    shooter.setVelocity(angularVelocityRad, AngleUnit.RADIANS);
                    sleep(1000);
                    gate.setPosition(shooterdump);
                    sleep(1000);
                    color_pattern.remove(0);
                    gate.setPosition(neutral);
                    shooter.setVelocity(0, AngleUnit.RADIANS);
                } else {
                    gate.setPosition(dumping);
                    sleep(1000);
                    gate.setPosition(neutral);
                    sleep(1000);
                }
                telemetry.update();
            }
        }
    }

    // --- AUTON SEQUENCE (BLUE-side with PID calls) ---
    @Override
    public void runOpMode() throws InterruptedException {
        Commons.init(hardwareMap, this::opModeIsActive, telemetry);
        telemetry.setMsTransmissionInterval(100);
        waitForStart();

        PID_forward(44, 0.5);        // Forward movement
        sleep(500);

        PID_rotateLeft(90, 0.5);     // Reverse rotation for blue
        sleep(500);

        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            LLResultTypes.FiducialResult fr = (LLResultTypes.FiducialResult) result.getFiducialResults();
            motif_pattern = fr.getFiducialId();
        }


        PID_rotateLeft(90, 0.5);     // Rotate more to align
        sleep(500);

        ty = result.getTy();
        sorter(motif_pattern, getdistance(ty));

        PID_rotateRight(30, 0.40);   // Rotate opposite for alignment
        sleep(500);

        intake.setPower(1);
        PID_forward(30, 0.75);
        sleep(500);
        intake.setPower(0);

        PID_backward(29, 0.75);
        sleep(500);

        PID_rotateRight(45, 0.45);
        ty = result.getTy();
        sorter(motif_pattern, getdistance(ty));

        PID_rotateLeft(45, 0.45);
        sleep(500);

        lateralLeft(23, 0.75);       // Strafe left for blue
        sleep(500);

        intake.setPower(1);
        sleep(500);
        PID_forward(30, 0.75);
        sleep(500);
        intake.setPower(0);


    }
}