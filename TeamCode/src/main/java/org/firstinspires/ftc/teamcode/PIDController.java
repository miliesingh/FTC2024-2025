package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    protected double kp;
    protected double ki;
    protected double kd;
    protected double integralSum = 0.0;
    protected double lastError = 0.0;
    protected double previousFilterEstimate = 0.0;
    protected ElapsedTime timer;

    /**
     * Construct the PID controller
     * @param p - Proportional coefficient
     * @param i - Integral coefficient
     * @param d - Derivative coefficient
     */
    public PIDController(double p, double i, double d) {
        kp = p;
        ki = i;
        kd = d;
        timer = new ElapsedTime();
    }
    /**
     * Update the PID output
     * @param target where we would like to be, also called the reference
     * @param state where we currently are, I.E. motor position
     */
    public double update (double target, double state) {
        double error;
        double errorChange;
        double derivative;
        double a = 0.707;
        double currentFilterEstimate;
        double result;

        // The sign is backwards because centralOffset is negative of the power we need.
        error = target - state;
        errorChange = error - lastError;

        currentFilterEstimate = (a * previousFilterEstimate) + (1-a) * errorChange;
        previousFilterEstimate = currentFilterEstimate;

        derivative = currentFilterEstimate / timer.seconds();
        integralSum = integralSum + (error * timer.seconds());

        result = (kp * error) + (ki * integralSum) + (kd * derivative);
        lastError = error;
        timer.reset();

        return result;
    }
}