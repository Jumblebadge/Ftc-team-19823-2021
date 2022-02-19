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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@Config
@TeleOp(name="Decentswerve", group="Linear Opmode")
public class Decentswerve extends LinearOpMode {


    private AnalogInput BLE = null;
    private AnalogInput BRE = null;
    private AnalogInput FLE = null;
    private AnalogInput FRE = null;

    private CRServo BLT = null;
    private CRServo BRT = null;
    private CRServo FLT = null;
    private CRServo FRT = null;

    private DcMotorEx BLD = null;
    private DcMotorEx BRD = null;
    private DcMotorEx FLD = null;
    private DcMotorEx FRD = null;

    private CRServo INS = null;
    private Servo INFL = null;
    private DcMotorEx INE = null;
//    private Servo DROP = null;

    private Servo OTD = null;
    private DcMotorEx OTE = null;
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
    double x2 = 0;
    public static double INFP = 0.5;
    public static double OTDP = 0.5;
    public static double DROPP = 0.5;

    public static double BLPC = 15;
    public static double FRPC = 5;
    public static double BRPC = -3;
    public static double FLPC = 2;

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

        BLD = hardwareMap.get(DcMotorEx.class, "BLD");
        BRD = hardwareMap.get(DcMotorEx.class, "BRD");
        FLD = hardwareMap.get(DcMotorEx.class, "FLD");
        FRD = hardwareMap.get(DcMotorEx.class, "FRD");

        BLT = hardwareMap.get(CRServo.class, "BLT");
        BRT = hardwareMap.get(CRServo.class, "BRT");
        FLT = hardwareMap.get(CRServo.class, "FLT");
        FRT = hardwareMap.get(CRServo.class, "FRT");

        INS = hardwareMap.get(CRServo.class, "INS");
        INFL = hardwareMap.get(Servo.class, "INFL");
        INE = hardwareMap.get(DcMotorEx.class,"INE");

//        DROP = hardwareMap.get(Servo.class, "DROP");

        OTD = hardwareMap.get(Servo.class, "OTD");
        OTE = hardwareMap.get(DcMotorEx.class,"OTE");
        BLT.setDirection(CRServo.Direction.REVERSE);
        BRT.setDirection(CRServo.Direction.REVERSE);
        FLT.setDirection(CRServo.Direction.REVERSE);
        FRT.setDirection(CRServo.Direction.REVERSE);
        INS.setDirection(DcMotorSimple.Direction.REVERSE);

        dashboard = FtcDashboard.getInstance();


        OTD.setPosition(0.2);
        //DROP.setPosition(DROPP);


        waitForStart();

        while (opModeIsActive()) {
            y2 = gamepad1.right_stick_y;
            x1 = gamepad1.left_stick_x;
            y1 = gamepad1.left_stick_y;
            x2 = gamepad1.right_stick_x;

            angles   = IMU.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double heading = angles.firstAngle*-1;

            INFP = Range.clip(INFP,0.1,1);

            if (gamepad1.x){
                y2 = -gamepad1.right_stick_y;
                x1 = gamepad1.left_stick_x;

                BRTreference = -45;
                BLTreference = 45;
                FRTreference = 45;
                FLTreference = -45;

                BLD.setPower(y2*-1/2);
                BRD.setPower(y2/2);
                FRD.setPower(y2/2);
                FLD.setPower(y2*-1/2);
            }
            else if (gamepad1.dpad_right){
                y2 = -gamepad1.right_stick_y;
                x1 = gamepad1.left_stick_x;

                BRTreference = 18;
                BLTreference = 18;
                FRTreference = -35;
                FLTreference = -35;

                BLD.setPower(y2);
                BRD.setPower(y2);
                FRD.setPower(y2);
                FLD.setPower(y2);
            }
            else if (gamepad1.dpad_left){
                y2 = -gamepad1.right_stick_y;
                x1 = gamepad1.left_stick_x;

                BRTreference = -18;
                BLTreference = -18;
                FRTreference = 35;
                FLTreference = 35;

                BLD.setPower(y2);
                BRD.setPower(y2);
                FRD.setPower(y2);
                FLD.setPower(y2);
            }
            else {
                y2 = -gamepad1.right_stick_y;
                x1 = gamepad1.left_stick_x;
                y1 = gamepad1.left_stick_y;
                if (y1 == 0 && x1 == 0){
                    y1 = -1;
                }

                double atan = Math.atan2(-x1,-y1);
                atan *= 57.295779513082320876;
                angles   = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = angles.firstAngle*-1;
                BLTreference = atan+heading;
                BRTreference = atan+heading;
                FLTreference = atan+heading;
                FRTreference = atan+heading;

                FRTreference -= FRPC;
                FLTreference -= FLPC;
                BRTreference -= BRPC;
                BLTreference -= BLPC;



                BLD.setPower(y2);
                BRD.setPower(y2);
                FRD.setPower(y2);
                FLD.setPower(y2);
            }

            if (gamepad2.a){
                INFP = 0.95;
            }
            if (!gamepad2.a) {
                if (INFP != 0.1){
                    INFP -= 0.029;
                }
            }
            INFL.setPosition(INFP);

            if (gamepad2.y){
                OTDP = 0.95;
            }
            if (!gamepad2.y){
                OTDP = 0.4;
            }
            OTD.setPosition(OTDP);
            //depositing pos = 0
            //resting pos = 0.4
            //init pos = 0.1

            double OTEV = 0;
            OTEV = gamepad2.right_trigger*-1;
            if (gamepad2.right_bumper){
                OTEV *= -1;
            }
            OTE.setPower(OTEV);

            double INEV = 0;
            INEV = gamepad2.left_trigger*-1;
            if (gamepad2.left_bumper){
                INEV *= -1;
            }
            INE.setPower(INEV);


            BLP = BLE.getVoltage() * 74.16;
            BRP = BRE.getVoltage() * 74.16;
            FLP = FLE.getVoltage() * 74.16;
            FRP = FRE.getVoltage() * 74.16;

            if(INFP>0.2){
                INS.setPower(1);}
            else if(gamepad2.b){
                INS.setPower(-0.25);}
            else{
                INS.setPower(0);}



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
            OTE.setPower(gamepad2.right_stick_y);
            String color = "#0f2259";
            String color1 = "#b28c00";
            telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
            telemetry.addData("", String.format("<span style=\"color:%s\">%s</span>",color1,"\n" +
                    "▬▬▬▬▬▬▬▬▬▬") + String.format("<span style=\"color:%s\">%s</span>",color,"" +"▬▬▬▬▬▬▬▬▬▬\n") +
                    String.format("<span style=\"color:%s\">%s</span>",color,"" +"░░░░░██╗██████╗░") + String.format("<span style=\"color:%s\">%s</span>",color1,"" +"|-----------------------------|\n") +
                    String.format("<span style=\"color:%s\">%s</span>",color,"" +"░░░░░██║██╔══██╗") + String.format("<span style=\"color:%s\">%s</span>",color1,"" +"|-----------------------------|\n") +
                    String.format("<span style=\"color:%s\">%s</span>",color,"" +"░░░░░██║██████╦╝") + String.format("<span style=\"color:%s\">%s</span>",color1,"" +"|------Jolly Blue--------|\n") +
                    String.format("<span style=\"color:%s\">%s</span>",color,"" +"██╗░░██║██╔══██╗") + String.format("<span style=\"color:%s\">%s</span>",color1,"" +"|------①⑨⑧②③-------|\n") +
                    String.format("<span style=\"color:%s\">%s</span>",color,"" +"╚█████╔╝██████╦╝") + String.format("<span style=\"color:%s\">%s</span>",color1,"" +"|--------Taylor------------|\n") +
                    String.format("<span style=\"color:%s\">%s</span>",color,"" +"░╚════╝░╚═════╝░") + String.format("<span style=\"color:%s\">%s</span>",color1,"" +"|--------Connor----------|\n") +
                    String.format("<span style=\"color:%s\">%s</span>",color1,"" +"▬▬▬▬▬▬▬▬▬▬") + String.format("<span style=\"color:%s\">%s</span>",color,""+"▬▬▬▬▬▬▬▬▬▬\n"));
            telemetry.update();
        }
    }
    }
