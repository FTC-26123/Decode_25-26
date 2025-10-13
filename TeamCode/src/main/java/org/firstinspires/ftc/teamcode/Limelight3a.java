package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp
public class Limelight3a extends LinearOpMode {

    private Limelight3A limelight;

//    private DcMotor frontLeftMotor;
//    private DcMotor frontRightMotor;
//    private DcMotor backLeftMotor;
//    private DcMotor backRightMotor;

    private boolean autoAlignPressedLast = false;

    // Limelight setup (temporary until robot is ready)
    final double limelightMountAngleDegrees = 90; // degrees
    final double limelightLensHeightInches = 13.5; // inches
    final double goalHeightInches = 40; // inches

    final double apriltagheight = 25;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize drivetrain motors
//        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
//        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
//        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
//        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            double tx = 0;
            double ty = 0;
            Pose3D botpose = null;
            double distanceFromLimelightToGoalInches = 0;
            double shooterVelocity = -1; // default invalid

            if (result != null && result.isValid()) {
                tx = result.getTx();
                ty = result.getTy();
                botpose = result.getBotpose();

                // Calculate distance
                double angleToGoalDegrees = limelightMountAngleDegrees + ty;
                double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
                distanceFromLimelightToGoalInches = (apriltagheight - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

                // Calculate shooter velocity
                double launchAngleDegrees = 45.0; // shooter angle/Temporary
                double launchHeight = 10.0; // height of ball exit in inches/Temporary
                double fudgeFactor = 0.3; // 30% extra power
                shooterVelocity = calculateShooterVelocity(distanceFromLimelightToGoalInches, launchHeight, goalHeightInches, launchAngleDegrees, fudgeFactor);

                // Telemetry
                telemetry.addData("tx", tx);
                telemetry.addData("ty", ty);
                telemetry.addData("Botpose", botpose);
                telemetry.addData("Distance (inches)", distanceFromLimelightToGoalInches);
                if (shooterVelocity > 0) {
                    telemetry.addData("Shooter v0 (in/s)", shooterVelocity);
                } else {
                    telemetry.addData("Shooter v0", "Invalid shot");
                }

                // Show fiducials
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                if (!fiducialResults.isEmpty()) {
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f",
                                fr.getFiducialId(), fr.getFamily(),
                                fr.getTargetXDegrees(), fr.getTargetYDegrees());
                        int apriltagid = fr.getFiducialId();
                        if(apriltagid == 21) {
                            telemetry.addLine("Pattern: GREEN, PURPLE, PURPLE(GPP)");
                        }
                        else if(apriltagid == 22) {
                            telemetry.addLine("Pattern: PURPLE, GREEN, PURPLE(PGP)");
                        }
                        else if(apriltagid == 23) {
                            telemetry.addLine("Pattern: PURPLE, PURPLE, GREEN(PPG)");
                        }
                        else if(apriltagid == 24) {
                            telemetry.addLine("Red Goal");
                        }
                        else if(apriltagid == 20) {
                            telemetry.addLine("Blue Goal");
                        }
                    }
                }
            } else {
                telemetry.addData("Limelight", "No valid target");
            }
            if (gamepad1.b) {
                //move forward the distance
            }





            boolean autoAlignPressed = gamepad1.a;
            if (autoAlignPressed && !autoAlignPressedLast && result != null && result.isValid()) {
                double kP = 0.02;
                double tolerance = 1.0;
                double maxPower = 0.3;

                if (Math.abs(tx) > tolerance) {
                    double turnPower = kP * tx;
                    turnPower = Math.max(-maxPower, Math.min(turnPower, maxPower));
                    telemetry.addLine("turn right");
                } else {
                    telemetry.addLine("turn left");
                }
            }
            autoAlignPressedLast = autoAlignPressed;

            telemetry.update();
        }
    }

    // Shooter velocity calculation
    private double calculateShooterVelocity(double distance, double launchHeight, double goalHeight, double launchAngleDegrees, double k) {
        double g = 386.0; // gravity in in/s^2
        double theta = Math.toRadians(launchAngleDegrees);
        double y = goalHeight - launchHeight;

        double denominator = 2 * Math.pow(Math.cos(theta), 2) * (distance * Math.tan(theta) - y);
        if (denominator <= 0) return -1; // invalid shot

        return Math.sqrt((g * distance * distance) / denominator) * (1 + k);
    }
}
