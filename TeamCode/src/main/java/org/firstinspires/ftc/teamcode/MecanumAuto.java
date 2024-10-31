package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Autonomous(name = "Mecanum Drive to Position", group = "Autonomous")
public class MecanumAuto extends BaseLinearOpMode {
    private static final double kP = 1, kI = 0, kD = .25; // Tweak these values
    private PIDController pidX, pidY, pidHeading;
    private double currentHeading;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        pidX = new PIDController(kP, kI, kD);
        pidY = new PIDController(kP, kI, kD);
        pidHeading = new PIDController(kP, kI, kD);

        waitForStart();

        // Move to (x, y) = (2, 2) at heading = 90 degrees
        //moveToPosition(2, 2, 90);
        moveTest(500);
    }

    public void moveToPosition(double deltaX, double deltaY, double targetHeading) {
        pidX.reset();
        pidY.reset();
        pidHeading.reset();
        while (opModeIsActive()) {
            updatePosition();
            currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double outputX = pidX.update(deltaX, 0);
            double outputY = pidY.update(deltaY, 0);
            double outputHeading = pidHeading.update(targetHeading, 0);

            // Calculate motor powers
            double frontLeftPower = outputY + outputX + outputHeading;
            double frontRightPower = outputY - outputX - outputHeading;
            double backLeftPower = outputY - outputX + outputHeading;
            double backRightPower = outputY + outputX - outputHeading;

            // Normalize powers
            double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }

            // Set motor powers
            topLeft.setPower(frontLeftPower);
            topRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            // Exit the loop when the robot is close enough to the target
            if (Math.abs(curPoseX - deltaX) < 0.1 && Math.abs(curPoseY - deltaY) < 0.1 && Math.abs(currentHeading - targetHeading) < 2.5) {
                break;
            }

            telemetry.addData("Target", "X: %.2f Y: %.2f Heading: %.2f", deltaX, deltaY, targetHeading);
            telemetry.addData("Current", "X: %.2f Y: %.2f Heading: %.2f", curPoseX, curPoseY, currentHeading);
            telemetry.update();
        }
        topLeft.setPower(0);
        topRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void moveTest(double target) { // single motor PID; tuning test
        pidX.reset();
        while (opModeIsActive()) {
            double output = pidX.update(target, topLeft.getCurrentPosition());
            topLeft.setPower(output);

            if (Math.abs(target - topLeft.getCurrentPosition()) < 2) {
                break;
            }

            telemetry.addData("Target: ", target);
            telemetry.addData("Current: ", topLeft.getCurrentPosition());
            telemetry.update();
        }
    }
}
