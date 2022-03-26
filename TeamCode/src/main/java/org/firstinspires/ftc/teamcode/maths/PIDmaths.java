package org.firstinspires.ftc.teamcode.maths;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class PIDmaths {

    private double integralSum;
    //PID for all things
    public double PIDout(double reference, double state, double Kp, double Kd, double Ki, double Kf, double elapstedtime) {
        //calculate error and normalize it
        double error = AngleUnit.normalizeDegrees(reference - state);
        double out=0;
        if (Math.abs(error) > 0) {
            error = AngleUnit.normalizeDegrees(reference - state);
            double lastError = 0;
            //calculate the integral and derivative values
            double derivative = (error - lastError) / elapstedtime;
            integralSum = integralSum + (error * elapstedtime);
            //multiplies those values by a constant, pre-tuned for whatever we are controlling
            out = (Kp * error) + (Kd * derivative) + (Ki * integralSum) + (Kf * Math.signum(error));
            lastError = error;
        }
        return out;
    }
}
