package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Mecanum Drive to Position", group = "Autonomous")
public class MecanumAuto extends BaseLinearOpMode {
    private static final double kP = 0.05, kI = 0.0, kD = 0.1; // Tune these values for better performance
    private PIDController pidX, pidY, pidHeading;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize PID controllers for X, Y, and heading
        pidX = new PIDController(kP, kI, kD);
        pidY = new PIDController(kP, kI, kD);
        pidHeading = new PIDController(kP, kI, kD);

        waitForStart();

        // Example usage: Move to a target position (x = 24 inches, y = 36 inches, heading = 90 degrees)
        moveToPosition(24.0, 36.0, Math.toRadians(90.0));
    }

    /**
     * Moves the robot to a target position (x, y) and heading using PID control.
     *
     * @param xTarget     Target X position in inches.
     * @param yTarget     Target Y position in inches.
     * @param thetaTarget Target heading in radians.
     */
    public void moveToPosition(double xTarget, double yTarget, double thetaTarget) {
        double tolerance = 0.5; // Tolerance for position and heading in inches/radians
        double maxPower = 0.5; // Maximum motor power to avoid excessive speed

        while (opModeIsActive()) {
            // Update the robot's current position and heading
            updateRobotPosition();

            // Get the current state of the robot
            double xRobotPosition = getRobotX();
            double yRobotPosition = getRobotY();
            double thetaRobotPosition = getRobotTheta();

            // Calculate PID outputs for X, Y, and heading
            double xOutput = pidX.calculate(xTarget, xRobotPosition);
            double yOutput = pidY.calculate(yTarget, yRobotPosition);
            double thetaOutput = pidHeading.calculate(thetaTarget, thetaRobotPosition);

            // Rotate the X and Y outputs to account for the robot's current heading
            double cosTheta = Math.cos(thetaRobotPosition);
            double sinTheta = Math.sin(thetaRobotPosition);
            double xRotated = xOutput * cosTheta - yOutput * sinTheta;
            double yRotated = xOutput * sinTheta + yOutput * cosTheta;

            // Combine the rotated outputs with the heading correction
            double powerTopLeft = xRotated + yRotated + thetaOutput;
            double powerBackLeft = xRotated - yRotated + thetaOutput;
            double powerTopRight = xRotated - yRotated - thetaOutput;
            double powerBackRight = xRotated + yRotated - thetaOutput;

            // Normalize motor powers to ensure they are within the range [-maxPower, maxPower]
            double maxMotorPower = Math.max(
                    Math.max(Math.abs(powerTopLeft), Math.abs(powerBackLeft)),
                    Math.max(Math.abs(powerTopRight), Math.abs(powerBackRight))
            );
            if (maxMotorPower > maxPower) {
                powerTopLeft = (powerTopLeft / maxMotorPower) * maxPower;
                powerBackLeft = (powerBackLeft / maxMotorPower) * maxPower;
                powerTopRight = (powerTopRight / maxMotorPower) * maxPower;
                powerBackRight = (powerBackRight / maxMotorPower) * maxPower;
            }

            // Apply the calculated powers to the motors
            topLeft.setPower(powerTopLeft);
            backLeft.setPower(powerBackLeft);
            topRight.setPower(powerTopRight);
            backRight.setPower(powerBackRight);

            // Exit the loop if the robot is close enough to the target position and heading
            if (Math.abs(xTarget - xRobotPosition) < tolerance &&
                    Math.abs(yTarget - yRobotPosition) < tolerance &&
                    Math.abs(thetaTarget - thetaRobotPosition) < tolerance) {
                break;
            }

            // Add a small delay to prevent excessive CPU usage
            sleep(10);
        }

        // Stop the motors once the target is reached
        topLeft.setPower(0);
        backLeft.setPower(0);
        topRight.setPower(0);
        backRight.setPower(0);

        // Reset PID controllers for future use
        pidX.reset();
        pidY.reset();
        pidHeading.reset();
    }
}