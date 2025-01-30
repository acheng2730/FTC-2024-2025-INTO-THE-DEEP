package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "scoop test")
public class scoop_test extends LinearOpMode {
    DcMotorEx scoop;

    @Override
    public void runOpMode() throws InterruptedException {
        scoop = hardwareMap.get(DcMotorEx.class, "scoop");
        double curPos = scoop.getCurrentPosition();
        waitForStart();

        while (opModeIsActive()) {
            double power = gamepad1.right_trigger - gamepad1.left_trigger;
            if ((scoop.getCurrentPosition() >= curPos && power > 0) ||
                    (scoop.getCurrentPosition() <= curPos - 225 && power < 0)) {
                power = 0; // Stop the motor if it tries to move beyond the limits
            }

            scoop.setPower(.35 * power);

            telemetry.addData("curPos: ", curPos);
            telemetry.addData("position: ", scoop.getCurrentPosition());
            telemetry.update();
        }
    }
}
