package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Autonomous(name = "Mecanum Drive to Position", group = "Autonomous")
public class MecanumAuto extends BaseLinearOpMode {
    private static final double kP = 1, kI = 0, kD = .25; //TODO Tweak these values
    private PIDController pidX, pidY, pidHeading;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        telemetry.addData("test", "");
        telemetry.update();

        pidX = new PIDController(kP, kI, kD);
        pidY = new PIDController(kP, kI, kD);
        pidHeading = new PIDController(kP, kI, kD);

        waitForStart();

    }

    public void moveTo(double xTarget, double yTarget, double thetaTarget) {
        double tolerance = 0.01; // Replace with your own tolerance value

        while (opModeIsActive()) {
            // Get current state of the robot
            double xRobotPosition = getRobotX();
            double yRobotPosition = getRobotY();
            double thetaRobotPosition = getRobotTheta();

            // Calculate control outputs
            double x = pidX.calculate(xTarget, xRobotPosition);
            double y = pidY.calculate(yTarget, yRobotPosition);
            double t = pidHeading.calculate(thetaTarget, thetaRobotPosition);

            // Rotate the vector
            double angle = Math.toRadians(45.0); // Replace with the current orientation of the robot if necessary
            double x_rotated = x * Math.cos(angle) - y * Math.sin(angle);
            double y_rotated = x * Math.sin(angle) + y * Math.cos(angle);

            double powerTopLeft = x_rotated + y_rotated + t;
            double powerBackLeft = x_rotated - y_rotated + t;
            double powerTopRight = x_rotated - y_rotated - t;
            double powerBackRight = x_rotated + y_rotated - t;

            // Find the maximum absolute power value
            double maxPower = Math.max(Math.abs(powerTopLeft), Math.max(Math.abs(powerBackLeft), Math.max(Math.abs(powerTopRight), Math.abs(powerBackRight))));

            // Normalize the power values to be within -1 and 1
            if (maxPower > 1) {
                powerTopLeft /= maxPower;
                powerBackLeft /= maxPower;
                powerTopRight /= maxPower;
                powerBackRight /= maxPower;
            }   

            // Apply control outputs to the motors
            topLeft.setPower(powerTopLeft);
            backLeft.setPower(powerBackLeft);
            topRight.setPower(powerTopRight);
            backRight.setPower(powerBackRight);

            // Exit the loop if the robot is close enough to the target
            if (Math.abs(xTarget - xRobotPosition) < tolerance && Math.abs(yTarget - yRobotPosition) < tolerance && Math.abs(thetaTarget - thetaRobotPosition) < tolerance) {
                pidX.reset();
                pidY.reset();
                pidHeading.reset();
                break;
            }
        }
    }
}