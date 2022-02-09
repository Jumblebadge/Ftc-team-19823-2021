package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;


@TeleOp(name="TestingRandomStuff", group="Linear Opmode")
public class TestingRandomStuff extends LinearOpMode {

    double BLTreference = 0;
    double BRTreference = 0;
    double FLTreference = 0;
    double FRTreference = 0;
    double x1 = 0;
    double y1 = 0;


    BNO055IMU IMU;
    Orientation angles;


    public void runOpMode() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //composeTelemetry();
        IMU = hardwareMap.get(BNO055IMU.class, "IMU");
        IMU.initialize(parameters);

        waitForStart();
        IMU.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        while (opModeIsActive()) {
            x1 = gamepad1.left_stick_x;
            y1 = gamepad1.left_stick_y;
            if (y1 == 0 && x1 == 0){
                y1 = -1;
            }

            double atan = Math.atan2(x1,-y1);
            atan *= 57.295779513082320876;
            angles   = IMU.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double heading = angles.firstAngle*-1;
            BLTreference = atan-heading;
            BRTreference = atan-heading;
            FLTreference = atan-heading;
            FRTreference = atan-heading;
            telemetry.addData("BLTreference",BLTreference);
            telemetry.addData("BRTreference",BRTreference);
            telemetry.addData("FLTreference",FLTreference);
            telemetry.addData("FRTreference",FRTreference);
            telemetry.addData("heading",heading);
            telemetry.addData("y1",y1);
            telemetry.addData("x1",x1);
            telemetry.update();
        }


    }}
    /**
    void composeTelemetry() {
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            angles   = IMU.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return IMU.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return IMU.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle*-1);

                    }
                })
                ;
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
        }

        String formatDegrees(double degrees){
            return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
        }
    }
**/


