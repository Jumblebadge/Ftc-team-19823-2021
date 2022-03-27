package org.firstinspires.ftc.teamcode.maths;


public class swerveMaths {
    //where our swerve math will be done
    public double[] Math(double forward, double strafe, double rotate, double imu, boolean fieldcentrictoggle){

        //define our math variables
        double strafe1,forward1,a,b,c,d;
        final double length,width,radius;

        //define our output variables
        double backRightSpeed,backLeftSpeed,backRightAngle,backLeftAngle,frontRightSpeed,frontLeftSpeed,frontRightAngle,frontLeftAngle;

        //depending on the orientation of your imu and wheels, you may have to reverse these values
        imu*=-1;
        forward*=-1;

        //field centric toggle
        if(fieldcentrictoggle==false){
            imu=0;
        }

        //distance between wheels
        length = 14.25;
        width = 14.25;
        //"radius" of wheels from center of robot
        radius = Math.sqrt((width * width) + (length * length));

        //rotate vectors by imu heading for field centric
        strafe1=Math.cos(Math.toRadians(imu))*strafe-Math.sin(Math.toRadians(imu))*forward;
        forward1=Math.sin(Math.toRadians(imu))*strafe+Math.cos(Math.toRadians(imu))*forward;

        //finding out what has to be done in x and y (length and width) for the movement you need
        a = strafe1 - rotate * (length / radius);
        b = strafe1 + rotate * (length / radius);
        c = forward1 - rotate * (width / radius);
        d = forward1 + rotate * (width / radius);

        //converting what the robot has to do into wheel specific values (speed)
        backRightSpeed = Math.sqrt((a * a) + (c * c));
        backLeftSpeed = Math.sqrt((a * a) + (d * d));
        frontRightSpeed = Math.sqrt((b * b) + (c * c));
        frontLeftSpeed = Math.sqrt((b * b) + (d * d));

        //make sure that values are scaled correctly and are under 1
        double max1 = Math.max(Math.abs(backLeftSpeed), Math.abs(backRightSpeed));
        double max2 = Math.max(max1, Math.abs(frontLeftSpeed));
        double max = Math.max(max2,Math.abs(frontRightSpeed));
        if(Math.abs(max)>1){
            backRightSpeed/=Math.abs(max);
            backLeftSpeed/=Math.abs(max);
            frontRightSpeed/=Math.abs(max);
            frontLeftSpeed/=Math.abs(max);
        }

        //converting what the robot has to do into wheel specific values (angle)
        backRightAngle = Math.atan2(a,c)*180 / Math.PI;
        backLeftAngle = Math.atan2(a,d)*180 / Math.PI;
        frontRightAngle = Math.atan2(b,c)*180 / Math.PI;
        frontLeftAngle = Math.atan2(b,d)*180/ Math.PI;

        backRightSpeed*=-1;
        backLeftSpeed*=-1;
        frontLeftSpeed*=-1;
        frontRightSpeed*=-1;


        //put our outputs into a array
        double[] output = {backRightSpeed,backLeftSpeed,frontRightSpeed,frontLeftSpeed,backRightAngle,backLeftAngle,frontRightAngle,frontLeftAngle};

        return output;
    }
}
