package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Tutorial")
public class Tutorial extends LinearOpMode {
    Servo one;
    Servo two;

    @Override
    public void runOpMode() throws InterruptedException {
        one = hardwareMap.get(Servo.class, "servo");
        two = hardwareMap.get(Servo.class, "servo2");
        one.setDirection(Servo.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.a) {
                one.setPosition(one.getPosition()+.01);
                two.setPosition(two.getPosition()+.01);
            }
            if(gamepad1.b) {
                one.setPosition(one.getPosition()-.01);
                two.setPosition(two.getPosition()-.01);
            }

            telemetry.addData("one: ", one.getPosition());
            telemetry.addData("two: ", two.getPosition());
            telemetry.update();
        }
    }
}
