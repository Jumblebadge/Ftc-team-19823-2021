package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="PIDTESTER", group="Linear Opmode")
public class PIDTESTER extends LinearOpMode {

    private DcMotorEx motortest;

    static double speed = 1200;

    public static PIDFCoefficients pidCoeffs = new PIDFCoefficients(1, 0, 0,0);
    public PIDFCoefficients pidGains = new PIDFCoefficients(1,0,0,0);
    ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    //@Override
    public void runOpMode() {

        motortest = hardwareMap.get(DcMotorEx.class,"motortest");

        motortest.setDirection(DcMotorSimple.Direction.REVERSE);

        motortest.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motortest.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();


        while (opModeIsActive()) {


            PID(speed);

            telemetry.update();

        }
    }

    double integral = 0;
    double lastError = 0;



    public void PID(double targetVelocity) {

        PIDTimer.reset();

        double currentVelocity =  motortest.getVelocity();


       double error = targetVelocity - currentVelocity;

       integral += integral*PIDTimer.time();

       double deltaError = error - lastError;
       double derivative = lastError/PIDTimer.time();

       pidGains.p = pidCoeffs.p*error;
       pidGains.i = pidCoeffs.i*integral;
       pidGains.d = pidCoeffs.d*derivative;

       motortest.setVelocity(pidGains.p+ pidGains.d+ pidGains.i+targetVelocity);

       lastError = error;

    }

}

