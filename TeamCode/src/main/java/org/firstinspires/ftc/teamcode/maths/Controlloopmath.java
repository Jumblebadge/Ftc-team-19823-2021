package org.firstinspires.ftc.teamcode.maths;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Controlloopmath {

    private double integralSum,out,lastError;
    private double Kp, Kd, Ki, Kf;
    private ElapsedTime elapsedtime;

    public Controlloopmath(double Kp, double Kd, double Ki, double Kf, ElapsedTime elapsedtime){
        this.elapsedtime=elapsedtime;
        this.Kp=Kp;
        this.Kd=Kd;
        this.Ki=Ki;
        this.Kf=Kf;
    }
    //PID loop
    public double PIDout(double reference, double state) {
        //calculate error and normalize it
        double error = AngleUnit.normalizeDegrees(reference - state);
        if (Math.abs(error) > 0) {
            error = AngleUnit.normalizeDegrees(reference - state);
            //calculate the integral and derivative values
            double derivative = (error - lastError) / elapsedtime.seconds();
            integralSum = integralSum + (error * elapsedtime.seconds());
            //multiplies those values by a constant, pre-tuned for whatever we are controlling
            out = (Kp * error) + (Kd * derivative) + (Ki * integralSum) + (Kf * Math.signum(error));
            out/=10;
            lastError = error;
            elapsedtime.reset();
        }
        return out;
    }
}
