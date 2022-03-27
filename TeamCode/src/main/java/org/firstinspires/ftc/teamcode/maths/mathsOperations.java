package org.firstinspires.ftc.teamcode.maths;

public class mathsOperations {

    //turns 0 through 360 degrees into 0 through 180 and -180 degrees
    public static double angleWrap(double wrap){

        while(wrap <= -180) {
            wrap += 360;
        }
        while(wrap > 180) {
            wrap -= 360;
        }
        return wrap;
    }

    public double efficientTurn(double reference,double state,double power){
        double error = reference-state;
        while(error>90) {
            reference = -180 + error;
            power *=-1;
        }
        while(error<-90) {
            reference = 180 + error;
            power *=-1;
        }

        double output = reference;
        return output;
    }
}
