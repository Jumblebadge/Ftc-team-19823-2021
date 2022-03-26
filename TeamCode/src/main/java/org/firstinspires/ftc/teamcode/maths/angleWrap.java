package org.firstinspires.ftc.teamcode.maths;

public class angleWrap {

    public static double angleWrap(double wrap){

        if(wrap <= -180) {
            wrap += 360;
        }
        if(wrap > 180) {
            wrap -= 360;
        }

        return wrap;
    }


}
