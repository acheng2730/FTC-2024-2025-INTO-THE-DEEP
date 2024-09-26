package org.firstinspires.ftc.teamcode;

public class PIDController {
    private final double kp;
    private final double ki;
    private final double kd;
    private double integral, previousError;
    private long lastTime;

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.integral = 0;
        this.previousError = 0;
        this.lastTime = System.currentTimeMillis();
    }

    public double update(double setpoint, double actualPosition) {
        long currentTime = System.currentTimeMillis();
        double elapsedTime = (currentTime - lastTime) / 1000.0; // Convert ms to s
        lastTime = currentTime;

        double error = setpoint - actualPosition;
        integral += error * elapsedTime;
        integral = Math.max(-2, Math.min(2, integral));
        double derivative = (error - previousError) / elapsedTime;

        previousError = error;

        return kp * error + ki * integral + kd * derivative;
    }

    public void reset() {
        integral = 0;
        previousError = 0;
        lastTime = System.currentTimeMillis();
    }
}
