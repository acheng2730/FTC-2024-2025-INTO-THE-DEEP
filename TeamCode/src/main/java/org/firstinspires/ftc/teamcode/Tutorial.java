package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "LinearTeleop")
public class Tutorial extends BaseLinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        while (opModeIsActive()) {
            topLeft.setPower(gamepad1.left_stick_y);
        }
    }
}
