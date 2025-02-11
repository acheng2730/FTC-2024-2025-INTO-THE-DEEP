package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BaseLinearOpMode;

@TeleOp(name = "Servo Fix")
public class servoFixing extends BaseLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //initHardware();
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");
        servoRight.setDirection(Servo.Direction.REVERSE);

        servoLeft.setPosition(0.5);
        servoRight.setPosition(0.5);

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

            telemetry.addData("left: ", servoLeft.getPosition());
            telemetry.addData("right: ", servoRight.getPosition());
            telemetry.update();
        }

    }
}
