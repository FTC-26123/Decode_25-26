package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name="Limelight3A_Iterative")
public class limelight extends OpMode {

    private Limelight3A limelight;
    private boolean autoAlignPressedLast = false;

    // Constants
    final double limelightMountAngleDegrees = 135;
    final double limelightLensHeightInches = 15;
    final double goalHeightInches = 40;
    final double apriltagheight = 25;

    @Override
    public void init() {
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.setMsTransmissionInterval(11);
        telemetry.addLine("Limelight initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        double tx = 0;
        double ty = 0;
        Pose3D botpose = null;
        double distanceFromLimelightToGoalInches = 0;
        double shooterVelocity = -1;

        if (result != null && result.isValid()) {
            tx = result.getTx();
            ty = result.getTy();
            botpose = result.getBotpose();

            // Calculate distance
            double angleToGoalDegrees = limelightMountAngleDegrees + ty;
            double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
            distanceFromLimelightToGoalInches = (apriltagheight - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

            // Calculate shooter velocity
            double launchAngleDegrees = 45.0;
            double launchHeight = 10.0;
            double fudgeFactor = 0.3;
            shooterVelocity = calculateShooterVelocity(
                    distanceFromLimelightToGoalInches,
                    launchHeight,
                    goalHeightInches,
                    launchAngleDegrees,
                    fudgeFactor
            );

            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.addData("Botpose", botpose);
            telemetry.addData("Distance (inches)", distanceFromLimelightToGoalInches);
            telemetry.addData("Shooter v0", shooterVelocity > 0 ? shooterVelocity : "Invalid shot");

            // Display fiducials
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            if (!fiducialResults.isEmpty()) {
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f",
                            fr.getFiducialId(), fr.getFamily(),
                            fr.getTargetXDegrees(), fr.getTargetYDegrees());

                    int apriltagid = fr.getFiducialId();
                    switch (apriltagid) {
                        case 21:
                            telemetry.addLine("Pattern: GREEN, PURPLE, PURPLE (GPP)");
                            break;
                        case 22:
                            telemetry.addLine("Pattern: PURPLE, GREEN, PURPLE (PGP)");
                            break;
                        case 23:
                            telemetry.addLine("Pattern: PURPLE, PURPLE, GREEN (PPG)");
                            break;
                        case 24:
                            telemetry.addLine("Red Goal");
                            break;
                        case 20:
                            telemetry.addLine("Blue Goal");
                            break;
                    }
                }
            }
        } else {
            telemetry.addData("Limelight", "No valid target");
        }

        // Handle auto-align button
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

    private double calculateShooterVelocity(double distance, double launchHeight, double goalHeight, double launchAngleDegrees, double k) {
        double g = 386.0; // gravity in in/s^2
        double theta = Math.toRadians(launchAngleDegrees);
        double y = goalHeight - launchHeight;

        double denominator = 2 * Math.pow(Math.cos(theta), 2) * (distance * Math.tan(theta) - y);
        if (denominator <= 0) return -1;

        return Math.sqrt((g * distance * distance) / denominator) * (1 + k);
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
        telemetry.addLine("OpMode stopped");
        telemetry.update();
    }
}
