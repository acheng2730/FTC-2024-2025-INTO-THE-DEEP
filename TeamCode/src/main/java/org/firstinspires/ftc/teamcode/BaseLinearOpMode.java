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
    public Servo servoLeft;
    public Servo servoRight;
    DcMotorEx topLeft, topRight, backLeft, backRight;
    DcMotorEx viper_slide;
    DcMotorEx scoop;
    double conversionFactor = 92.4; // NeveRest 40 motor ticks/inch
    double curPoseY = 0, curPoseX = 0;
    double curTheta = 0; double encoderTheta = 0; // Current position on field in inches
    ElapsedTime driveTime = new ElapsedTime();
    double prevTime = 0;
    Servo wrist;

    //TODO FIX THESE NUMBERS
    private static final double WHEEL_RADIUS = 2.0; // Wheel radius in inches
    private static final double LX = 7.0; // Half of the wheelbase along the x-axis in inches
    private static final double LY = 7.0; // Half of the wheelbase along the y-axis in inches

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

    public double getRobotX() {
        return curPoseX;
    }

    public double getRobotY() {
        return curPoseY;
    }

    public double getRobotTheta() {
        return curTheta;
    }

    // Method to update the robot's position and heading
    public void updateRobotPosition() {
        double currentTime = driveTime.seconds();
        double deltaTime = currentTime - prevTime;
        prevTime = currentTime;

        // Get encoder values (in inches)
        double w1 = topLeft.getCurrentPosition() / conversionFactor;
        double w2 = topRight.getCurrentPosition() / conversionFactor;
        double w3 = backLeft.getCurrentPosition() / conversionFactor;
        double w4 = backRight.getCurrentPosition() / conversionFactor;

        // Calculate robot velocities using mecanum wheel kinematics
        double vx = (WHEEL_RADIUS / 4) * (w1 + w2 + w3 + w4);
        double vy = (WHEEL_RADIUS / 4) * (-w1 + w2 + w3 - w4);
        double omega = (WHEEL_RADIUS / (4 * (LX + LY))) * (-w1 + w2 - w3 + w4);

        encoderTheta = omega*deltaTime;
        encoderTheta = normalizeAngle(encoderTheta);

        // Update the robot's heading using the IMU
        curTheta = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        /*
         * optional filter for imu + encoder heading
        curTheta = .9 * curTheta + .1 * encoderTheta;
         */

        // Update the robot's position in the global frame
        curPoseX += (vx * Math.cos(curTheta) - vy * Math.sin(curTheta)) * deltaTime;
        curPoseY += (vx * Math.sin(curTheta) + vy * Math.cos(curTheta)) * deltaTime;
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    @Override
    public abstract void runOpMode() throws InterruptedException;

}
