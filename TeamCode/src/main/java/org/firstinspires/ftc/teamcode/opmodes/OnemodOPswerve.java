package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Config
@TeleOp(name="OnemodOPswerve", group="Linear Opmode")
public class OnemodOPswerve extends LinearOpMode {


    private AnalogInput BLE = null;
    private AnalogInput BRE = null;
    private AnalogInput FLE = null;
    private AnalogInput FRE = null;

    private CRServo BLT = null;
    private CRServo BRT = null;
    private CRServo FLT = null;
    private CRServo FRT = null;

    //private DcMotorEx BLD = null;
    //private DcMotorEx BRD = null;
    //private DcMotorEx FLD = null;
    //private DcMotorEx FRD = null;

    FtcDashboard dashboard;

    public static double Kp = 0.2;
    public static double Ki = 0;
    public static double Kd = 0.0001;
    public static double Kf = 0;

    public static double BLTreference = 0;
    double BLTintegralSum = 0;
    double BLTlastError = 0;
    double BLTerror = 0;

    public static double BRTreference = 0;
    double BRTintegralSum = 0;
    double BRTlastError = 0;
    double BRTerror = 0;

    public static double FLTreference = 0;
    double FLTintegralSum = 0;
    double FLTlastError = 0;
    double FLTerror = 0;

    public static double FRTreference = 0;
    double FRTintegralSum = 0;
    double FRTlastError = 0;
    double FRTerror = 0;

    public static double tolerance = 0;

    ElapsedTime BLTtimer =  new ElapsedTime();
    ElapsedTime BRTtimer =  new ElapsedTime();
    ElapsedTime FLTtimer =  new ElapsedTime();
    ElapsedTime FRTtimer =  new ElapsedTime();

    double BLP = 0;
    double BRP = 0;
    double FLP = 0;
    double FRP = 0;


    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        BLE = hardwareMap.get(AnalogInput.class, "BLE");
        BRE = hardwareMap.get(AnalogInput.class, "BRE");
        FLE = hardwareMap.get(AnalogInput.class, "FLE");
        FRE = hardwareMap.get(AnalogInput.class, "FRE");

        //BLD = hardwareMap.get(DcMotorEx.class, "BLD");
        //BRD = hardwareMap.get(DcMotorEx.class, "BRD");
        //FLD = hardwareMap.get(DcMotorEx.class, "FLD");
        //FRD = hardwareMap.get(DcMotorEx.class, "FRD");

        BLT = hardwareMap.get(CRServo.class, "BLT");
        BRT = hardwareMap.get(CRServo.class, "BRT");
        FLT = hardwareMap.get(CRServo.class, "FLT");
        FRT = hardwareMap.get(CRServo.class, "FRT");

        BLT.setDirection(CRServo.Direction.REVERSE);
        //BRT.setDirection(CRServo.Direction.REVERSE);
        FLT.setDirection(CRServo.Direction.REVERSE);
        //FRT.setDirection(CRServo.Direction.REVERSE);


        dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {

            final double L = 14.25;
            final double W = 14.25;

            double r = Math.sqrt((L * L) + (W * W));
            gamepad1.left_stick_y *= -1;

            double a = gamepad1.left_stick_x - gamepad1.right_stick_x * (L / r);
            double b = gamepad1.left_stick_x + gamepad1.right_stick_x * (L / r);
            double c = gamepad1.left_stick_y - gamepad1.right_stick_x * (W / r);
            double d = gamepad1.left_stick_y + gamepad1.right_stick_x * (W / r);

            double backRightSpeed1 = Math.sqrt((a * a) + (d * d));
            double backLeftSpeed1 = Math.sqrt((a * a) + (c * c));
            double frontRightSpeed1 = Math.sqrt((b * b) + (d * d));
            double frontLeftSpeed1 = Math.sqrt((b * b) + (c * c));

            double backRightAngle1 = Math.atan2(a, d);
            double backLeftAngle1 = Math.atan2(a, c);
            double frontRightAngle1 = Math.atan2(b, d);
            double frontLeftAngle1 = Math.atan2(b, c);

            double backRightSpeed = backRightSpeed1;
            double backLeftSpeed = backLeftSpeed1;
            double frontRightSpeed = frontRightSpeed1;
            double frontLeftSpeed = frontLeftSpeed1;

            double backRightAngle = backRightAngle1 * 180;
            double backLeftAngle = backLeftAngle1 * 180;
            double frontRightAngle = frontRightAngle1 * 180;
            double frontLeftAngle = frontLeftAngle1 * 180;

            BLTreference = backLeftAngle;
            BRTreference = backRightAngle;
            FLTreference = frontLeftAngle;
            FRTreference = frontRightAngle;

            BLP = BLE.getVoltage() * 74.16;
            BRP = BRE.getVoltage() * 74.16;
            FLP = FLE.getVoltage() * 74.16;
            FRP = FRE.getVoltage() * 74.16;


            if(BLP <= -180) {
                BLP += 360;
            }
            if(BLP > 180) {
                BLP -= 360;
            }

            if(BRP <= -180) {
                BRP += 360;
            }
            if(BRP > 180) {
                BRP -= 360;
            }

            if(FLP <= -180) {
                FLP += 360;
            }
            if(FLP > 180) {
                FLP -= 360;
            }

            if(FRP <= -180) {
                FRP += 360;
            }
            if(FRP > 180) {
                FRP -= 360;
            }

            //BLD.setPower(gamepad1.right_stick_y);
            //BRD.setPower(gamepad1.right_stick_y);
            //FRD.setPower(gamepad1.right_stick_y);
            // FLD.setPower(gamepad1.right_stick_y);

            BLTerror = AngleUnit.normalizeDegrees(BLTreference - BLP);
            if (Math.abs(BLTerror) > tolerance ) {
                BLTerror = AngleUnit.normalizeDegrees(BLTreference - BLP);

                double BLTderivative = (BLTerror - BLTlastError) / BLTtimer.seconds();
                BLTintegralSum = BLTintegralSum + (BLTerror * BLTtimer.seconds());
                double BLTout = (Kp*BLTerror)+(Kd*BLTderivative)+(Ki*BLTintegralSum)+(Kf*Math.signum(BLTerror));
                BLT.setPower(BLTout/10);

                BLTlastError = BLTerror;
                BLTtimer.reset();

            }
            if (Math.abs(BLTerror)<tolerance){
                BLT.setPower(0);
            }


            BRTerror = AngleUnit.normalizeDegrees(BRTreference- BRP);
            if (Math.abs(BRTerror) > tolerance ) {
                BRTerror = AngleUnit.normalizeDegrees(BRTreference- BRP);

                double BRTderivative = (BRTerror - BRTlastError) / BRTtimer.seconds();
                BRTintegralSum = BRTintegralSum + (BRTerror * BRTtimer.seconds());
                double BRTout = (Kp*BRTerror)+(Kd*BRTderivative)+(Ki*BRTintegralSum)+(Kf*Math.signum(BRTerror));
                BRT.setPower(BRTout/10);

                BRTlastError = BRTerror;
                BRTtimer.reset();

            }
            if (Math.abs(BRTerror)<tolerance){
                BRT.setPower(0);
            }


            FLTerror = AngleUnit.normalizeDegrees(FLTreference - FLP);
            if (Math.abs(FLTerror) > tolerance ) {
                FLTerror = AngleUnit.normalizeDegrees(FLTreference - FLP);

                double FLTderivative = (FLTerror - FLTlastError) / FLTtimer.seconds();
                FLTintegralSum = FLTintegralSum + (FLTerror * FLTtimer.seconds());
                double FLTout = (Kp*FLTerror)+(Kd*FLTderivative)+(Ki*FLTintegralSum)+(Kf*Math.signum(FLTerror));
                FLT.setPower(FLTout/10);

                FLTlastError = FLTerror;
                FLTtimer.reset();

            }
            if (Math.abs(FLTerror)<tolerance){
                FLT.setPower(0);
            }


            FRTerror = AngleUnit.normalizeDegrees(FRTreference - FRP);
            if (Math.abs(FRTerror) > tolerance ) {

                FRTerror = AngleUnit.normalizeDegrees(FRTreference - FRP);

                double FRTderivative = (FRTerror - FRTlastError) / FRTtimer.seconds();
                FRTintegralSum = FRTintegralSum + (FRTerror * FRTtimer.seconds());
                double FRTout = (Kp*FRTerror)+(Kd*FRTderivative)+(Ki*FRTintegralSum)+(Kf*Math.signum(FRTerror));
                FRT.setPower(FRTout/10);

                FRTlastError = FRTerror;
                FRTtimer.reset();

            }
            if (Math.abs(FRTerror)<tolerance){
                FRT.setPower(0);
            }
            telemetry.addData("BLTreference",BLTreference);
            telemetry.addData("BRTreference",BRTreference);
            telemetry.addData("FLTreference",FLTreference);
            telemetry.addData("FRTreference",FRTreference);
            telemetry.addData("BLP",BLP);
            telemetry.addData("BRP",BRP);
            telemetry.addData("FLP",FLP);
            telemetry.addData("FRP",FRP);

            telemetry.update();
        }
    }
}