package org.firstinspires.ftc.teamcode.old;

import android.companion.BluetoothLeDeviceFilter;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.AnalogInput;
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
@TeleOp(name="PIDTESTER4", group="Linear Opmode")
public class PIDTESTER4 extends LinearOpMode {

    private CRServo BLD;
    private AnalogInput BLE;


    double Kp = 2;
    double Ki = 0.3;
    double Kd = 0;

    double reference = 230;
    double integralSum = 0;
    double lastError = 0;
    double tolerance = 10;
    double error = 0;

    ElapsedTime timer =  new ElapsedTime();
    //@Override
    public void runOpMode() throws InterruptedException {

        BLD = hardwareMap.get(CRServo.class,"BLD");
        BLE = hardwareMap.get(AnalogInput.class,"BLE");

        waitForStart();


        while (opModeIsActive()) {
            double curposdeg = BLE.getVoltage()*74.16;
            error = reference - curposdeg;
            if (Math.abs(error) > tolerance ) {


                curposdeg = BLE.getVoltage()*74.16;
                error = reference - curposdeg;


                double derivative = (error - lastError) / timer.seconds();
                integralSum = integralSum + (error * timer.seconds());
                double out = (Kp*error)+(Kd*error);
                BLD.setPower(out);

                lastError = error;
                timer.reset();
                telemetry.addData("error",error);
                telemetry.addData("curposdeg",curposdeg);
                telemetry.update();

            }
            if (Math.abs(error)<tolerance){

                //telemetry.addData("doneeeeee","oh yeah");
                BLD.setPower(0);
            }








        }


    }

}
