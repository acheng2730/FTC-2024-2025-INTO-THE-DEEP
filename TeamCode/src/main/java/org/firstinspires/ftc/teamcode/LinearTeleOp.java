package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// A more accurate implementation of Mecanum drive, using target angle/power as inputs instead of direct joystick values
@TeleOp(name = "LinearTeleOp")
public class LinearTeleOp extends BaseLinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        servoRight.setPosition(1);
        servoLeft.setPosition(0.45);
        wrist.setPosition(0);

        PIDController viper = new PIDController(.004,0.004,0);
        int viperSetpoint = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        int scoopPos = scoop.getCurrentPosition();
        waitForStart();

        while (opModeIsActive()) {
            int topLeftEncoderPos = topLeft.getCurrentPosition();
            int topRightEncoderPos = -topRight.getCurrentPosition();
            int backLeftEncoderPos = -backLeft.getCurrentPosition();
            int backRightEncoderPos = backRight.getCurrentPosition();
            int viperEncoderPos = -viper_slide.getCurrentPosition();


            //updatePosition();
            getRobotTheta();
            getRobotX();
            getRobotY();

            telemetry.addData("Position: ", curPoseX + " , " + curPoseY + " , " + curTheta);
            telemetry.addData("topLeftPos: ", topLeftEncoderPos);
            telemetry.addData("topRightPos: ", topRightEncoderPos);
            telemetry.addData("backLeftPos: ", backLeftEncoderPos);
            telemetry.addData("backRightPos: ", backRightEncoderPos);
            telemetry.addData("viperPos: ", viperEncoderPos);
            telemetry.addData("viper setpoint: ", viperSetpoint);
            telemetry.addData("scoop pos: ", scoop.getCurrentPosition());
            telemetry.addData("servoLeft: ", servoLeft.getPosition());
            telemetry.addData("servoRight: ", servoRight.getPosition());
            telemetry.addData("wrist: ", wrist.getPosition());

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
            double viperPower = 0;

            if ((power + Math.abs(turn)) > 1) { // Avoid power clipping
                topLeftPow /= power + turn;
                backLeftPow /= power + turn;
                topRightPow /= power + turn;
                backRightPow /= power + turn;
            }

            if (gamepad1.left_bumper) {
                viperSetpoint -= 20;
            }
            if (gamepad1.right_bumper) {
                viperSetpoint += 20;
            }

            if (viperSetpoint <=-50) {
                viperSetpoint = -50;
            }
            if (viperSetpoint >= 2725) {
                viperSetpoint = 2725;
            }
            viperPower = viper.calculate(viperSetpoint, viperEncoderPos);

            if (gamepad1.a) {
                servoLeft.setPosition(servoLeft.getPosition()+.02);
                servoRight.setPosition(servoRight.getPosition()+.02);
                sleep(10);
            }
            if (gamepad1.b) {
                servoLeft.setPosition(servoLeft.getPosition()-.02);
                servoRight.setPosition(servoRight.getPosition()-.02);
                sleep(10);
            }

            if (gamepad1.x) {
                wrist.setPosition(wrist.getPosition()+.02);
                sleep(10);
            }
            if (gamepad1.y) {
                wrist.setPosition(wrist.getPosition()-.02);
                sleep(10);
            }

            double scoopPower = gamepad1.right_trigger-gamepad1.left_trigger;

            topLeft.setPower(topLeftPow);
            backLeft.setPower(backLeftPow);
            topRight.setPower(topRightPow);
            backRight.setPower(backRightPow);
            viper_slide.setPower(viperPower);
            scoop.setPower(.35*scoopPower);
        }
    }
}