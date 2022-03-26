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

import org.firstinspires.ftc.teamcode.maths.mathsOperations;
import org.firstinspires.ftc.teamcode.maths.swerveMaths;
import org.firstinspires.ftc.teamcode.maths.PIDmaths;


@Config
@TeleOp(name="Godswerve", group="Linear Opmode")
public class Godswerve extends LinearOpMode {

    private AnalogInput BLE = null, BRE = null, FLE = null, FRE = null;

    private CRServo BLT = null, BRT = null, FLT = null, FRT = null;

    private DcMotorEx BLD = null, BRD = null, FLD = null, FRD = null;

    FtcDashboard dashboard;

    double BLTreference = 0, BRTreference=0,FLTreference=0,FRTreference=0;

    ElapsedTime BLTtimer =  new ElapsedTime();
    ElapsedTime BRTtimer =  new ElapsedTime();
    ElapsedTime FLTtimer =  new ElapsedTime();
    ElapsedTime FRTtimer =  new ElapsedTime();

    double BLP = 0, BRP = 0, FLP = 0, FRP = 0;

    double x1 = 0, y2 = 0, y1 = 0, x2 = 0;

    public static double BLPC = 10, FRPC = -5, BRPC = -8, FLPC = -10;

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

        BLT.setDirection(CRServo.Direction.REVERSE);
        BRT.setDirection(CRServo.Direction.REVERSE);
        FLT.setDirection(CRServo.Direction.REVERSE);
        FRT.setDirection(CRServo.Direction.REVERSE);

        dashboard = FtcDashboard.getInstance();

        swerveMaths swavemath = new swerveMaths();
        PIDmaths pidmath = new PIDmaths();


        waitForStart();

        while (opModeIsActive()) {
            y2 = gamepad1.right_stick_y;
            x1 = gamepad1.left_stick_x;
            y1 = gamepad1.left_stick_y;
            x2 = gamepad1.right_stick_x;

            angles   = IMU.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double heading = angles.firstAngle*-1;

            swavemath.Math(gamepad1.left_stick_y, gamepad1.left_stick_x,gamepad1.right_stick_x,heading,true);

            BLP = BLE.getVoltage() * 74.16;
            BRP = BRE.getVoltage() * 74.16;
            FLP = FLE.getVoltage() * 74.16;
            FRP = FRE.getVoltage() * 74.16;

            mathsOperations.angleWrap(BLP);
            mathsOperations.angleWrap(BRP);
            mathsOperations.angleWrap(FLP);
            mathsOperations.angleWrap(FRP);

            mathsOperations.angleWrap(BLTreference);
            mathsOperations.angleWrap(BRTreference);
            mathsOperations.angleWrap(FLTreference);
            mathsOperations.angleWrap(FRTreference);

            BLT.setPower(pidmath.PIDout(BLTreference,BLP,0.2,0.0001,0,0,BLTtimer.seconds()));
            BLTtimer.reset();

            BRT.setPower(pidmath.PIDout(BRTreference,BRP,0.2,0.0001,0,0,BRTtimer.seconds()));
            BRTtimer.reset();

            FLT.setPower(pidmath.PIDout(FLTreference,FLP,0.2,0.0001,0,0,FLTtimer.seconds()));
            FLTtimer.reset();

            FRT.setPower(pidmath.PIDout(FRTreference,FRP,0.2,0.0001,0,0,FRTtimer.seconds()));
            FRTtimer.reset();



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
            telemetry.addData("IMU",heading);

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
