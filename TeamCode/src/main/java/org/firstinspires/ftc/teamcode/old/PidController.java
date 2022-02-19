package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class PidController {

    public double PidController(double reference, double degree, double Kp, double Kd){

        ElapsedTime timer = new ElapsedTime();

        double lastError = 0;
        double error = reference - degree;

        double derivative = (error - lastError) / timer.seconds();
        double out = (Kp*error)+(Kd*error);
        //+derivative?

        lastError = error;
        timer.reset();

        return out;

    }
}
