package org.firstinspires.ftc.teamcode.maths;

public class swerveMaths {

    //where our swerve math will be done
    public double[] Math(double forward, double strafe, double rotate, double imu, boolean fieldcentrictoggle){

        //define our math variables
        double strafe1,forward1,m1x,m1y,m2x,m2y,m3x,m3y,m4x,m4y;

        //define our output variables
        double backRightSpeed,backLeftSpeed,backRightAngle,backLeftAngle,frontRightSpeed,frontLeftSpeed,frontRightAngle,frontLeftAngle;

        //depending on the orientation of your imu and wheels, you may have to reverse these values
        imu*=-1;
        forward*=-1;

        //field centric toggle
        if(fieldcentrictoggle==false){
            //imu=0;
        }

        //rotate vectors by imu heading for field centric
        strafe1=Math.cos(Math.toRadians(imu))*strafe-Math.sin(Math.toRadians(imu))*forward;
        forward1=Math.sin(Math.toRadians(imu))*strafe+Math.cos(Math.toRadians(imu))*forward;

        //making vectors for each wheel of what it has to do to achieve our movement goal (separated into x and y)
        m1x = strafe1 - rotate * 1;
        m2x = strafe1 - rotate * 1;
        m3x = strafe1 - rotate * -1;
        m4x = strafe1 - rotate * -1;

        m1y = forward1 + rotate * -1;
        m2y = forward1 + rotate * 1;
        m3y = forward1 + rotate * 1;
        m4y = forward1 + rotate * -1;

        //extracting speed from our vector (length of vector)
        backRightSpeed = Math.sqrt((m1x * m1x) + (m1y * m1y));
        backLeftSpeed = Math.sqrt((m2x * m2x) + (m2y * m2y));
        frontRightSpeed = Math.sqrt((m3x * m3x) + (m3y * m3y));
        frontLeftSpeed = Math.sqrt((m4x * m4x) + (m4y * m4y));

        //make sure that speed values are scaled correctly and are under 1
        double max1 = Math.max(Math.abs(backLeftSpeed), Math.abs(backRightSpeed));
        double max2 = Math.max(max1, Math.abs(frontLeftSpeed));
        double max = Math.max(max2,Math.abs(frontRightSpeed));
        if(Math.abs(max)>1){
            backRightSpeed/=Math.abs(max);
            backLeftSpeed/=Math.abs(max);
            frontRightSpeed/=Math.abs(max);
            frontLeftSpeed/=Math.abs(max);
        }

        //extracting angle from our vector (angle of vector face)
        backRightAngle = Math.atan2(m1y,m1x)*180 / Math.PI;
        backLeftAngle = Math.atan2(m2y,m2x)*180 / Math.PI;
        frontRightAngle = Math.atan2(m3y,m3x)*180 / Math.PI;
        frontLeftAngle = Math.atan2(m4y,m4x)*180/ Math.PI;

        backRightSpeed*=-1;
        backLeftSpeed*=-1;
        frontLeftSpeed*=-1;
        frontRightSpeed*=-1;

        //put our outputs into an array
        double[] output = {backRightSpeed,backLeftSpeed,frontRightSpeed,frontLeftSpeed,backRightAngle,backLeftAngle,frontRightAngle,frontLeftAngle};

        return output;
    }
}
