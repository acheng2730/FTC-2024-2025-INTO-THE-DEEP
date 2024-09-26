package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// A more accurate implementation of Mecanum drive, using target angle/power as inputs instead of direct joystick values
@TeleOp(name = "robotCentric-method2")
public class LinearTeleOp extends BaseLinearOpMode {

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
}