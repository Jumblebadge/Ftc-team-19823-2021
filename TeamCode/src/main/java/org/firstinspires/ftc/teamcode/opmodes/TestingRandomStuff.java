package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
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
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@Config
@TeleOp(name="TestingRandomStuff", group="Linear Opmode")
public class TestingRandomStuff extends LinearOpMode {


    private AnalogInput BLE = null;
    private AnalogInput BRE = null;
    private AnalogInput FLE = null;
    private AnalogInput FRE = null;

    private CRServo BLT = null;
    private CRServo BRT = null;
    private CRServo FLT = null;
    private CRServo FRT = null;


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

    double x1 = 0;
    double y2 = 0;
    double y1 = 0;

    double atan = 0;

    BNO055IMU IMU;
    Orientation angles;


    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        IMU = hardwareMap.get(BNO055IMU.class, "IMU");
        IMU.initialize(parameters);
        IMU.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        BLE = hardwareMap.get(AnalogInput.class, "BLE");
        BRE = hardwareMap.get(AnalogInput.class, "BRE");
        FLE = hardwareMap.get(AnalogInput.class, "FLE");
        FRE = hardwareMap.get(AnalogInput.class, "FRE");

        BLT = hardwareMap.get(CRServo.class, "BLT");
        BRT = hardwareMap.get(CRServo.class, "BRT");
        FLT = hardwareMap.get(CRServo.class, "FLT");
        FRT = hardwareMap.get(CRServo.class, "FRT");
        BLT.setDirection(CRServo.Direction.REVERSE);
        BRT.setDirection(CRServo.Direction.REVERSE);
        FLT.setDirection(CRServo.Direction.REVERSE);
        FRT.setDirection(CRServo.Direction.REVERSE);

        dashboard = FtcDashboard.getInstance();


        waitForStart();

        while (opModeIsActive()) {
            y2 = gamepad1.right_stick_y;
            x1 = gamepad1.left_stick_x;
            y1 = gamepad1.left_stick_y;


            angles   = IMU.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double heading = angles.firstAngle*-1;


            y2 = gamepad1.right_stick_y;
            x1 = gamepad1.left_stick_x;


            if (gamepad1.x){
                y2 = -gamepad1.right_stick_y;
                x1 = gamepad1.left_stick_x;

                BRTreference = -45;
                BLTreference = 45;
                FRTreference = 45;
                FLTreference = -45;
            }
            else if (gamepad1.dpad_right){
                y2 = -gamepad1.right_stick_y;
                x1 = gamepad1.left_stick_x;

                BRTreference = 18;
                BLTreference = 18;
                FRTreference = -35;
                FLTreference = -35;

            }
            else if (gamepad1.dpad_left){
                y2 = -gamepad1.right_stick_y;
                x1 = gamepad1.left_stick_x;

                BRTreference = -18;
                BLTreference = -18;
                FRTreference = 35;
                FLTreference = 35;

            }
            else {
                y2 = gamepad1.right_stick_y;
                x1 = gamepad1.left_stick_x;
                y1 = gamepad1.left_stick_y;
                if (y1 == 0 && x1 == 0){
                    y1 = -1;
                }

                double atan = Math.atan2(x1,-y1);
                atan *= 57.295779513082320876;
                angles   = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = angles.firstAngle*-1;
                BLTreference = atan-heading;
                BRTreference = atan-heading;
                FLTreference = atan-heading;
                FRTreference = atan-heading;


            }


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

            if(BLTreference <= -180) {
                BLTreference += 360;
            }
            if(BLTreference > 180) {
                BLTreference -= 360;
            }
            if(BRTreference <= -180) {
                BRTreference += 360;
            }
            if(BRTreference > 180) {
                BRTreference -= 360;
            }
            if(FLTreference <= -180) {
                FLTreference += 360;
            }
            if(FLTreference > 180) {
                FLTreference -= 360;
            }
            if(FRTreference <= -180) {
                FRTreference += 360;
            }
            if(FRTreference > 180) {
                FRTreference -= 360;
            }







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
            telemetry.addData("heading",heading);
            telemetry.addData("Atan",atan);
            telemetry.update();
        }
    }
}
