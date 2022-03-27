package org.firstinspires.ftc.teamcode.maths;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Controlloopmath {

    private double integralSum,out,lastError;
    //PID loop
    public double PIDout(double reference, double state, double Kp, double Kd, double Ki, double Kf, double elapstedtime) {
        //calculate error and normalize it
        double error = AngleUnit.normalizeDegrees(reference - state);
        if (Math.abs(error) > 0) {
            error = AngleUnit.normalizeDegrees(reference - state);
            //calculate the integral and derivative values
            double derivative = (error - lastError) / elapstedtime;
            integralSum = integralSum + (error * elapstedtime);
            //multiplies those values by a constant, pre-tuned for whatever we are controlling
            out = (Kp * error) + (Kd * derivative) + (Ki * integralSum) + (Kf * Math.signum(error));
            out/=10;
            lastError = error;
        }
        return out;
    }
}
