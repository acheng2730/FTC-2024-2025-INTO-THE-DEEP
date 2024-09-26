package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Tutorial")
public class Tutorial extends LinearOpMode {
    DcMotorEx topLeft;

    @Override
    public void runOpMode() throws InterruptedException {
        topLeft = hardwareMap.get(DcMotorEx.class, "topLeft");
        waitForStart();
        while (opModeIsActive()) {
            topLeft.setPower(gamepad1.left_stick_y);
        }
    }
}
