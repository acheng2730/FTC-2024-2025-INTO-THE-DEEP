package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// A more accurate implementation of Mecanum drive, using target angle/power as inputs instead of direct joystick values
@TeleOp(name = "robotCentric-method2")
public class LinearTeleOp extends BaseLinearOpMode {
    double curPoseY = 0, curPoseX = 0; // Current position on field in inches
    ElapsedTime driveTime = new ElapsedTime();
    double prevTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            int topLeftEncoderPos = topLeft.getCurrentPosition();
            int topRightEncoderPos = topRight.getCurrentPosition();
            int backLeftEncoderPos = backLeft.getCurrentPosition();
            int backRightEncoderPos = backRight.getCurrentPosition();

            updatePosition();

            telemetry.addData("Position: ", curPoseX + " , " + curPoseY);
            telemetry.addData("topLeftPos: ", topLeftEncoderPos);
            telemetry.addData("topRightPos: ", topRightEncoderPos);
            telemetry.addData("backLeftPos: ", backLeftEncoderPos);
            telemetry.addData("backRightPos: ", backRightEncoderPos);

            telemetry.update();

            // Mecanum drivetrain implementation
            // MUST READ: https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
            // https://www.youtube.com/watch?v=gnSW2QpkGXQ

            double strafe = gamepad1.left_stick_x * 1.1;
            double drive = gamepad1.left_stick_y * -1;
            double turn = gamepad1.right_stick_x;

            double theta = Math.atan2(drive, strafe); // Desired bot heading
            double power = Math.hypot(strafe, drive); // Desired power

            double sin = Math.sin(theta - Math.PI / 4);
            double cos = Math.cos(theta - Math.PI / 4); // Rotating the bot heading here by -45 degrees makes the
            // mecanum wheels' vectors the x,y components of the power vector
            double max = Math.max(Math.abs(sin), Math.abs(cos)); // Scale motors so at least one is max power

            double topLeftPow = power * cos / max + turn;
            double backLeftPow = power * sin / max + turn;
            double topRightPow = power * sin / max - turn;
            double backRightPow = power * cos / max - turn;

            if ((power + Math.abs(turn)) > 1) { // Avoid power clipping
                topLeftPow /= power + turn;
                backLeftPow /= power + turn;
                topRightPow /= power + turn;
                backRightPow /= power + turn;
            }

            topLeft.setPower(topLeftPow);
            backLeft.setPower(backLeftPow);
            topRight.setPower(topRightPow);
            backRight.setPower(backRightPow);

        }
    }

    public void updatePosition() { // uses encoders to determine position on the field
        // super inaccurate, most likely not going to use - also should be an auton only thing
        // MUST READ: https://ftc-tech-toolbox.vercel.app/docs/odo/Mecanum
        double angle = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        // apply mecanum kinematic model (with wheel velocities [ticks per sec])
        double xV = (topLeft.getVelocity() + topRight.getVelocity() + backLeft.getVelocity() + backRight.getVelocity()) * 0.482;

        double yV = (-topLeft.getVelocity() + topRight.getVelocity() + backLeft.getVelocity() - backRight.getVelocity()) * 0.482;

        // rotate the vector
        double nx = (xV * Math.cos(angle)) - (yV * Math.sin(angle));
        double nY = (xV * Math.sin(angle)) + (yV * Math.cos(angle));
        xV = nx;
        yV = nY;

        // integrate velocity over time
        curPoseY += (yV * (driveTime.seconds() - prevTime)) / conversionFactor; // <-- Tick to inch conversion factor
        curPoseX += (xV * (driveTime.seconds() - prevTime)) / conversionFactor;
        prevTime = driveTime.seconds();
    }
}