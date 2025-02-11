package org.firstinspires.ftc.teamcode;

public class PIDController {
    private final double kp; // Proportional gain
    private final double ki; // Integral gain
    private final double kd; // Derivative gain
    private double integral; // Integral sum
    private double previousError; // Previous error for derivative calculation
    private long lastTime; // Last time the controller was updated

    // Output clamping limits
    private double outputMin = -1.0; // Minimum output (e.g., -1.0 for motor power)
    private double outputMax = 1.0;  // Maximum output (e.g., 1.0 for motor power)

    // Integral windup protection
    private double integralMin = -1.0; // Minimum integral sum
    private double integralMax = 1.0;  // Maximum integral sum

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.integral = 0;
        this.previousError = 0;
        this.lastTime = System.nanoTime(); // Use nanoTime for better precision
    }

    /**
     * Calculates the PID output based on the setpoint and actual position.
     *
     * @param setpoint        The target value.
     * @param actualPosition  The current value.
     * @return The PID output.
     */
    public double calculate(double setpoint, double actualPosition) {
        long currentTime = System.nanoTime();
        double elapsedTime = (currentTime - lastTime) / 1e9; // Convert nanoseconds to seconds
        lastTime = currentTime;

        // Calculate error
        double error = setpoint - actualPosition;

        // Proportional term
        double proportional = kp * error;

        // Integral term with windup protection
        integral += error * elapsedTime;
        integral = Math.max(integralMin, Math.min(integralMax, integral)); // Clamp integral
        double integralTerm = ki * integral;

        // Derivative term (based on change in actual position to avoid derivative kick)
        double derivative = kd * (actualPosition - previousError) / elapsedTime;
        previousError = actualPosition;

        // Calculate output
        double output = proportional + integralTerm - derivative;

        // Clamp output to prevent excessive values
        output = Math.max(outputMin, Math.min(outputMax, output));

        return output;
    }

    /**
     * Resets the PID controller's internal state.
     */
    public void reset() {
        integral = 0;
        previousError = 0;
        lastTime = System.nanoTime(); // Reset time to avoid large elapsed time on next call
    }

    /**
     * Sets the output clamping limits.
     *
     * @param min Minimum output value.
     * @param max Maximum output value.
     */
    public void setOutputLimits(double min, double max) {
        this.outputMin = min;
        this.outputMax = max;
    }

    /**
     * Sets the integral windup protection limits.
     *
     * @param min Minimum integral sum.
     * @param max Maximum integral sum.
     */
    public void setIntegralLimits(double min, double max) {
        this.integralMin = min;
        this.integralMax = max;
    }
}