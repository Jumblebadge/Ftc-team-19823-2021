package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.acmerobotics.dashboard.FtcDashboard;



@Config
@TeleOp(name="OnemodOPswerve", group="Linear Opmode")
public class OnemodOPswerve extends LinearOpMode {

    private DcMotorEx BLD = null;
    private AnalogInput BLE = null;

    private CRServo BLT = null;

    double enccurpos = 0;
    double curposdeg = 0;
    double enccurposA = 0;

    FtcDashboard dashboard;

    public static double Kp = 4.175;
    public static double Ki = 0;
    public static double Kd = 0.3;
    public static double Kf = 0.25;

    public static double reference = 100;
    double integralSum = 0;
    double lastError = 0;
    public static double tolerance = 3;
    double error = 0;

    ElapsedTime timer =  new ElapsedTime();
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        BLE = hardwareMap.get(AnalogInput.class, "BLE");
        BLD = hardwareMap.get(DcMotorEx.class, "BLD");
        BLT = hardwareMap.get(CRServo.class, "BLT");
        BLD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLT.setDirection(CRServo.Direction.REVERSE);
        dashboard = FtcDashboard.getInstance();

        waitForStart();
        //14.25inL   14.25inW
        while (opModeIsActive()) {

            final double L = 14.25;
            final double W = 14.25;

            double r = Math.sqrt ((L * L) + (W * W));
            gamepad1.left_stick_y *= -1;

            double a = gamepad1.left_stick_x - gamepad1.right_stick_x * (L / r);
            double b = gamepad1.left_stick_x + gamepad1.right_stick_x * (L / r);
            double c = gamepad1.left_stick_y - gamepad1.right_stick_x * (W / r);
            double d = gamepad1.left_stick_y + gamepad1.right_stick_x * (W / r);

            double backRightSpeed1 = Math.sqrt ((a * a) + (d * d));
            double backLeftSpeed1 = Math.sqrt ((a * a) + (c * c));
            double frontRightSpeed1 = Math.sqrt ((b * b) + (d * d));
            double frontLeftSpeed1 = Math.sqrt ((b * b) + (c * c));

            double backRightAngle1 = Math.atan2 (a, d) / Math.PI;
            double backLeftAngle1 = Math.atan2 (a, c) / Math.PI;
            double frontRightAngle1 = Math.atan2 (b, d) / Math.PI;
            double frontLeftAngle1 = Math.atan2 (b, c) / Math.PI;


            double backRightSpeed = backRightSpeed1;
            double backLeftSpeed = backLeftSpeed1;
            double frontRightSpeed = frontRightSpeed1;
            double frontLeftSpeed = frontLeftSpeed1;

            double backRightAngle = backRightAngle1*180;
            double backLeftAngle = backLeftAngle1*180;
            double frontRightAngle = frontRightAngle1*180;
            double frontLeftAngle = frontLeftAngle1*180;


            telemetry.addData("backrightspeed",backRightSpeed);
            telemetry.addData("backleftspeeed",backLeftSpeed);
            telemetry.addData("frontrightspeed",frontRightSpeed);
            telemetry.addData("frontleftspeed",frontLeftSpeed);

            telemetry.addData("backrightangle",backRightAngle);
            telemetry.addData("backleftangle",backLeftAngle);
            telemetry.addData("frontrightangle",frontRightAngle);
            telemetry.addData("frontleftangle",frontLeftAngle);
            telemetry.addData("enccurpos",enccurpos);
            telemetry.addData("curposdeg",curposdeg);
            telemetry.update();



            curposdeg = BLE.getVoltage()*74.16;
            curposdeg = curposdeg%360 - (curposdeg%360 > 180 ? 360 : 0);
            enccurpos = BLE.getVoltage();
            reference = backLeftAngle;


            error = reference - curposdeg;

            if(curposdeg <= -180) {
                curposdeg += 360;
            }
            if(curposdeg > 180) {
                curposdeg -= 360;
            }
            //BLD.setPower(backLeftSpeed);

            if (Math.abs(error) > tolerance ) {

                reference = backLeftAngle;
                error = reference - curposdeg;

                if(curposdeg <= -180) {
                    curposdeg += 360;
                }
                if(curposdeg > 180) {
                    curposdeg -= 360;
                }

                double derivative = (error - lastError) / timer.seconds();
                integralSum = integralSum + (error * timer.seconds());
                double out = (Kp*error)+(Kd*derivative)+(Ki*integralSum)+(Kf*Math.signum(error));
                BLT.setPower(out);

                lastError = error;
                timer.reset();

                telemetry.addData("error",error);
                telemetry.addData("reference",reference);
                telemetry.addData("curposdeg",curposdeg);
                telemetry.addData("out", out);
                telemetry.update();


            }
            else if (Math.abs(error)<tolerance){

                //telemetry.addData("doneeeeee","oh yeah");
                BLT.setPower(0);
            }
        }
    }
}