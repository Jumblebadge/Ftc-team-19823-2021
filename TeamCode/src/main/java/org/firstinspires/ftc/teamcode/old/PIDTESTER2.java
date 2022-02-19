package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.io.InterruptedIOException;

@Disabled
@TeleOp(name="PIDTESTER2", group="Linear Opmode")
public class PIDTESTER2 extends LinearOpMode {

    private DcMotorEx motortest;

    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;

    ElapsedTime timer =  new ElapsedTime();
    private double lastError = 0;
    //@Override
    public void runOpMode() throws InterruptedException {

        motortest = hardwareMap.get(DcMotorEx.class,"motortest");

        motortest.setDirection(DcMotorSimple.Direction.REVERSE);

        motortest.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();


        while (opModeIsActive()) {


            double power = PIDControl(100, motortest.getCurrentPosition());
            motortest.setPower(power);
            telemetry.update();

        }
    }





    public double PIDControl(double reference, double state) {

        double error = reference - state;
        integralSum += error*timer.seconds();
        double derivative = (error - lastError) / timer.seconds();

        timer.reset();

        double output = (error*Kp) + (derivative*Kd) + (integralSum * Ki);
        return output;

    }

}

