package org.firstinspires.ftc.teamcode.maths;

public class swerveMaths {

    //where our swerve math will be done
    public double[] Math(double forward, double strafe, double rotate, double imu, boolean fieldcentrictoggle){

        forward*=-1;

        //distance between wheels
        double length = 14.25;
        double width = 14.25;

        //distance of wheels from center of robot
        double radius = Math.sqrt((width * width) + (length * length));

        //rotate vectors by imu heading for field centric
        double strafe1=Math.cos(Math.toRadians(imu))*strafe-Math.sin(Math.toRadians(imu))*forward;
        double forward1=Math.sin(Math.toRadians(imu))*strafe+Math.cos(Math.toRadians(imu))*forward;

        //finding out what has to be done in x and y (length and width) for the movement you need
        double a = strafe1 - rotate * (length / radius);
        double b = strafe1 + rotate * (length / radius);
        double c = forward1 - rotate * (width / radius);
        double d = forward1 + rotate * (width / radius);

        //converting what the robot has to do into wheel specific values (speed)
        double backRightSpeed = Math.sqrt((a * a) + (c * c));
        double backLeftSpeed = Math.sqrt((a * a) + (d * d));
        double frontRightSpeed = Math.sqrt((b * b) + (c * c));
        double frontLeftSpeed = Math.sqrt((b * b) + (d * d));

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
        double backRightAngle = Math.atan2(a,c)*180 / Math.PI;
        double backLeftAngle = Math.atan2(a,d)*180 / Math.PI;
        double frontRightAngle = Math.atan2(b,c)*180 / Math.PI;
        double frontLeftAngle = Math.atan2(b,d)*180/ Math.PI;

        //put our outputs into an array
        double[] output = {backRightSpeed,backLeftSpeed,frontRightSpeed,frontLeftSpeed,backRightAngle,backLeftAngle,frontRightAngle,frontLeftAngle};

        return output;
    }
}
