package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

// A working knowledge of Java is helpful here:
// abstract classes and inheritance
public abstract class BaseLinearOpMode extends LinearOpMode {
    public IMU imu;
    DcMotorEx topLeft, topRight, backLeft, backRight;
    DcMotorEx viper_slide;
    DcMotorEx scoop;
    double conversionFactor = 92.4; // NeveRest 40 motor ticks/inch
    double curPoseY = 0, curPoseX = 0; double curTheta = 0; // Current position on field in inches
    ElapsedTime driveTime = new ElapsedTime();
    double prevTime = 0;

    public Servo servoLeft;
    public Servo servoRight;
    Servo wrist;

    public void initHardware() throws InterruptedException {
        // Hubs
        List<LynxModule> allHubs;
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        topLeft = hardwareMap.get(DcMotorEx.class, "topLeft");
        topRight = hardwareMap.get(DcMotorEx.class, "topRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        viper_slide = hardwareMap.get(DcMotorEx.class, "viper");
        scoop = hardwareMap.get(DcMotorEx.class, "scoop");

        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");
        servoRight.setDirection(Servo.Direction.REVERSE);
        wrist = hardwareMap.get(Servo.class, "wrist");

        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE); // For mecanum drive

        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        scoop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        viper_slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        scoop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);
    }

    public double getRobotTheta() {
        curTheta = (int) -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return remap(curTheta, -180, 180, 0, 360);
    }

    public double getRobotY() {
        int topLeftEncoderPos = topLeft.getCurrentPosition();
        int topRightEncoderPos = -topRight.getCurrentPosition();
        int backLeftEncoderPos = -backLeft.getCurrentPosition();
        int backRightEncoderPos = backRight.getCurrentPosition();
        return curPoseY = (double) (backLeftEncoderPos + topLeftEncoderPos + backRightEncoderPos + topRightEncoderPos) / 4;
    }

    public double getRobotX() {
        int topLeftEncoderPos = topLeft.getCurrentPosition();
        int topRightEncoderPos = -topRight.getCurrentPosition();
        int backLeftEncoderPos = -backLeft.getCurrentPosition();
        int backRightEncoderPos = backRight.getCurrentPosition();
        return curPoseX = (double) (-backLeftEncoderPos - topLeftEncoderPos + backRightEncoderPos + topRightEncoderPos) / 4;
    }

    public void updatePosition() { // uses encoders to determine position on the field
        // super inaccurate, most likely not going to use - also should be an auton only thing
        // MUST READ: https://ftc-tech-toolbox.vercel.app/docs/odo/Mecanum
        double angle = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        // apply mecanum kinematic model (with wheel velocities [ticks per sec])
        double xV = (topLeft.getVelocity() + topRight.getVelocity() + backLeft.getVelocity() + backRight.getVelocity()) * 0.482;

        double yV = (-topLeft.getVelocity() + topRight.getVelocity() + backLeft.getVelocity() - backRight.getVelocity()) * 0.482;

        // rotate the vector
        double nx = (xV*Math.cos(angle))-(yV*Math.sin(angle));
        double nY = (xV*Math.sin(angle))+(yV*Math.cos(angle));
        xV = nx; yV = nY;

        // integrate velocity over time
        //curPoseY += (yV * (driveTime.seconds() - prevTime)) / conversionFactor; // <-- Tick to inch conversion factor
        //curPoseX += (xV * (driveTime.seconds() - prevTime)) / conversionFactor;
        prevTime = driveTime.seconds();
    }

    public static double remap(double value, double low, double high, double newLow, double newHigh) {
        return newLow + (newHigh - newLow) * ((value - low) / (high - low));
    }


    @Override
    public abstract void runOpMode() throws InterruptedException;

}
