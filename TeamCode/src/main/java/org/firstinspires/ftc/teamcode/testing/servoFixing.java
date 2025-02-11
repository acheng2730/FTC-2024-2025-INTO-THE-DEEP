package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BaseLinearOpMode;

@TeleOp(name = "Servo Fix")
public class servoFixing extends BaseLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        servoLeft.setPosition(0);
        servoRight.setPosition(0);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                servoLeft.setPosition(servoLeft.getPosition() + .005);
                servoRight.setPosition(servoRight.getPosition() + .005);
                sleep(10);
            }
            if (gamepad1.right_bumper) {
                servoLeft.setPosition(servoLeft.getPosition() - .005);
                servoRight.setPosition(servoRight.getPosition() - .005);
                sleep(10);
            }
        }

    }
}
