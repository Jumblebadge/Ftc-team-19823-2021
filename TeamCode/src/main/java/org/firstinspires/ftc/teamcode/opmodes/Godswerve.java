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

    public static double BLTreference = 0, BRTreference=0,FLTreference=0,FRTreference=0;

    ElapsedTime BLTtimer =  new ElapsedTime();
    ElapsedTime BRTtimer =  new ElapsedTime();
    ElapsedTime FLTtimer =  new ElapsedTime();
    ElapsedTime FRTtimer =  new ElapsedTime();

    double BLP = 0, BRP = 0, FLP = 0, FRP = 0;

    double x1 = 0, y2 = 0, y1 = 0, x2 = 0;

    double BLTpower=0,BRTpower=0,FLTpower=0,FRTpower=0,BLDpower,BRDpower,FLDpower,FRDpower;

    public static double BLPC = 10, FRPC = -5, BRPC = -8, FLPC = -10;
    public static double Kp=0.2,Ki=0,Kd=0.0001,Kf=0;

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
        mathsOperations maths = new mathsOperations();


        waitForStart();

        while (opModeIsActive()) {

            angles   = IMU.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double heading = angles.firstAngle*-1;

            double[] output = swavemath.Math(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x,heading,false);
            BRDpower=output[0];
            BLDpower=output[1];
            FRDpower=output[2];
            FLDpower=output[3];
            BRTreference=output[4];
            BLTreference=output[5];
            FRTreference=output[6];
            FLTreference=output[7];

            BLP = BLE.getVoltage() * 74.16;
            BRP = BRE.getVoltage() * 74.16;
            FLP = FLE.getVoltage() * 74.16;
            FRP = FRE.getVoltage() * 74.16;

            BLP=mathsOperations.angleWrap(BLP);
            BRP=mathsOperations.angleWrap(BRP);
            FLP=mathsOperations.angleWrap(FLP);
            FRP=mathsOperations.angleWrap(FRP);

            BLTreference=mathsOperations.angleWrap(BLTreference);
            BRTreference=mathsOperations.angleWrap(BRTreference);
            FLTreference=mathsOperations.angleWrap(FLTreference);
            FRTreference=mathsOperations.angleWrap(FRTreference);

            BLTreference= maths.efficientTurn(BLTreference,BLP,BLDpower);


            double BRTvalues = maths.efficientTurn(BRTreference,BRP,BRDpower);
            //BRTreference=BRTvalues[0];
            //BRDpower=BRTvalues[1];

            double FLTvalues = maths.efficientTurn(FLTreference,FLP,FLDpower);
            //FLTreference=FLTvalues[0];
            //FLDpower=FLTvalues[1];

            double FRTvalues = maths.efficientTurn(FRTreference,FRP,FRDpower);
            //FRTreference=FRTvalues[0];
            //FRDpower=FRTvalues[1];

            BLT.setPower(pidmath.PIDout(BLTreference,BLP,Kp,Kd,Ki,Kf,BLTtimer.seconds()));
            //BLD.setPower(BLDpower)
            BLTtimer.reset();

            BRT.setPower(pidmath.PIDout(BRTreference,BRP,Kp,Kd,Ki,Kf,BRTtimer.seconds()));
            //BRD.setPower(BRDpower)
            BRTtimer.reset();

            FLT.setPower(pidmath.PIDout(FLTreference,FLP,Kp,Kd,Ki,Kf,FLTtimer.seconds()));
            //FLD.setPower(FLDpower)
            FLTtimer.reset();

            FRT.setPower(pidmath.PIDout(FRTreference,FRP,Kp,Kd,Ki,Kf,FRTtimer.seconds()));
            //FRD.setPower(FRDpower)
            FRTtimer.reset();

            telemetry.addData("IMU",heading);

            telemetry.addData("BLTreference",BLTreference);
            telemetry.addData("BRTreference",BRTreference);
            telemetry.addData("FLTreference",FLTreference);
            telemetry.addData("FRTreference",FRTreference);

            telemetry.addData("BLP",BLP);
            telemetry.addData("BRP",BRP);
            telemetry.addData("FLP",FLP);
            telemetry.addData("FRP",FRP);

            telemetry.addData("FRDpower",FRDpower);
            telemetry.addData("FLDpower",FLDpower);
            telemetry.addData("BRDpower",BRDpower);
            telemetry.addData("BLDpower",BLDpower);
            telemetry.update();
        }
    }
}
