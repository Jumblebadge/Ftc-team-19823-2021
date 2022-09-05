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
    //makes sure no wheels turn more than they have to (keeps them within 90 degrees and reverses motor power to compensate)
    public static double[] efficientTurn(double reference,double state,double power){
        double error = reference-state;

        while(error>90) {
            power *=-1;
            reference -= 180;
            error = reference-state;
        }
        while(error<-90) {
            power *=-1;
            reference += 180;
            error = reference-state;
        }

        return new double[]{reference,power};
    }
    public static double[] diffyConvert(double rotate, double translate){
        double top = (rotate + translate) / 2.0;
        double bottom = (rotate - translate) / 2.0;
        return new double[]{top,bottom};
    }
}
