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
@TeleOp(name="PIDTESTER3", group="Linear Opmode")
public class PIDTESTER3 extends LinearOpMode {

    private DcMotorEx motortest;


    double Kp = 3;
    double Ki = 0;
    double Kd = 0;

    double reference = 5000;
    double integralSum = 0;
    double lastError = 0;
    double attemptpos = 65748;
    double tolerance = 75;

    ElapsedTime timer =  new ElapsedTime();
    //@Override
    public void runOpMode() throws InterruptedException {

        motortest = hardwareMap.get(DcMotorEx.class,"motortest");

        motortest.setDirection(DcMotorSimple.Direction.REVERSE);
        motortest.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motortest.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        waitForStart();


        while (opModeIsActive()) {
            double enccurpos = motortest.getCurrentPosition();
            double error = reference - enccurpos;

            if (Math.abs(error) > tolerance ) {
                enccurpos = motortest.getCurrentPosition();

                 error = reference - enccurpos;

                double derivative = (error - lastError) / timer.seconds();
                integralSum = integralSum + (error * timer.seconds());
                double out = (Kp*error)+(Kd*error)+(Ki*error);
                out = Range.clip(out, -1,1);
                motortest.setPower(out);

                lastError = error;
                timer.reset();
                telemetry.addData("out",out);
                telemetry.addData("lasterror",lastError);
                telemetry.addData("timer",timer.seconds());
                telemetry.addData("error",error);
                telemetry.addData("enccurpos",enccurpos);
                telemetry.addData("reference",reference);
                telemetry.addData("derivative",derivative);
                telemetry.addData("integralsum",integralSum);
                telemetry.addData("done?","no");
                telemetry.addData("encoderabs",Math.abs(error));
                telemetry.update();

            }
            else {

                telemetry.addData("doneeeeee","oh yeah");
                motortest.setPower(0);

            }




        }


    }

}

