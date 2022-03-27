package org.firstinspires.ftc.teamcode.opmodes;

//Import EVERYTHING we need
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;import com.acmerobotics.dashboard.config.Config;import com.qualcomm.robotcore.hardware.AnalogInput;import com.acmerobotics.dashboard.FtcDashboard;import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;import com.qualcomm.hardware.bosch.BNO055IMU;import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;import com.qualcomm.robotcore.hardware.DcMotorEx;import com.qualcomm.robotcore.hardware.CRServo;import com.qualcomm.robotcore.util.ElapsedTime;import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;import org.firstinspires.ftc.robotcore.external.navigation.Orientation;import org.firstinspires.ftc.teamcode.maths.Controlloopmath;import org.firstinspires.ftc.teamcode.maths.mathsOperations;import org.firstinspires.ftc.robotcore.external.navigation.Position;import org.firstinspires.ftc.teamcode.maths.swerveMaths;

import java.util.List;


@Config
@TeleOp(name="Godswerve", group="Linear Opmode")
public class Godswerve extends LinearOpMode {

    //Initialize all of our hardware
    private AnalogInput BLE = null, BRE = null, FLE = null, FRE = null;

    private CRServo BLT = null, BRT = null, FLT = null, FRT = null;

    private DcMotorEx BLD = null, BRD = null, FLD = null, FRD = null;

    List<LynxModule> allHubs = null;

    //Initialize FTCDashboard
    FtcDashboard dashboard;

    //Define reference points
    public static double BLTreference = 0, BRTreference=0,FLTreference=0,FRTreference=0;

    //Timers for the PID loops
    ElapsedTime BLTtimer =  new ElapsedTime(); ElapsedTime FRTtimer =  new ElapsedTime(); ElapsedTime FLTtimer =  new ElapsedTime(); ElapsedTime BRTtimer =  new ElapsedTime();

    //Define values for wheel positions
    double BLP = 0, BRP = 0, FLP = 0, FRP = 0;

    //Gamepad values
    double x1 = 0, y2 = 0, y1 = 0, x2 = 0;

    //Variables for power of wheels
    double BLDpower,BRDpower,FLDpower,FRDpower;

    //Tuning values so that wheels are always facing straight (accounts for encoder drift - tuned manually)
    public static double BLPC = -40, FRPC = -4, BRPC = 0, FLPC = -9;

    //PID values
    public static double Kp=0.2,Ki=0,Kd=0.0001,Kf=0;

    //IMU
    BNO055IMU IMU;
    Orientation angles;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Calibrate the IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        IMU = hardwareMap.get(BNO055IMU.class, "IMU");
        IMU.initialize(parameters);
        IMU.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //Initialize FTCDashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Link all of our hardware to our hardwaremap
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

        //Bulk sensor reads
        allHubs = hardwareMap.getAll(LynxModule.class);

        //Initialize FTCDashboard
        dashboard = FtcDashboard.getInstance();

        //Create objects for the classes we use for swerve
        swerveMaths swavemath = new swerveMaths();
        Controlloopmath pidmath = new Controlloopmath();

        //Bulk sensor reads
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


        waitForStart();

        while (opModeIsActive()) {

            //Clear the cache for better loop times (bulk sensor read)
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            //Turn our MA3 absolute encoder signals from volts to degrees
            BLP = BLE.getVoltage() * -74.16;
            BRP = BRE.getVoltage() * -74.16;
            FLP = FLE.getVoltage() * -74.16;
            FRP = FRE.getVoltage() * -74.16;

            //Update heading of robot
            angles   = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double heading = angles.firstAngle*-1;

            //Retrieve the angles and powers for all of our wheels from the swerve kinematics
            double[] output = swavemath.Math(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x,heading,true);
            BRDpower=output[0];
            BLDpower=output[1];
            FRDpower=output[2];
            FLDpower=output[3];
            BRTreference=output[4];
            BLTreference=output[5];
            FRTreference=output[6];
            FLTreference=output[7];

            //Subtract our tuning values to account for any encoder drift
            FRTreference -= FRPC;
            FLTreference -= FLPC;
            BRTreference -= BRPC;
            BLTreference -= BLPC;

            //Anglewrap our positions and references for each wheel
            BLP=mathsOperations.angleWrap(BLP);
            BRP=mathsOperations.angleWrap(BRP);
            FLP=mathsOperations.angleWrap(FLP);
            FRP=mathsOperations.angleWrap(FRP);

            BLTreference=mathsOperations.angleWrap(BLTreference);
            BRTreference=mathsOperations.angleWrap(BRTreference);
            FLTreference=mathsOperations.angleWrap(FLTreference);
            FRTreference=mathsOperations.angleWrap(FRTreference);

            //Run our powers, references, and positions through efficient turning code for each wheel and get the new values
            double[] BLTvalues= mathsOperations.efficientTurn(BLTreference,BLP,BLDpower);
            BLTreference=BLTvalues[0];
            BLDpower=BLTvalues[1];

            double[] BRTvalues = mathsOperations.efficientTurn(BRTreference,BRP,BRDpower);
            BRTreference=BRTvalues[0];
            BRDpower=BRTvalues[1];

            double[] FLTvalues = mathsOperations.efficientTurn(FLTreference,FLP,FLDpower);
            FLTreference=FLTvalues[0];
            FLDpower=FLTvalues[1];

            double[] FRTvalues = mathsOperations.efficientTurn(FRTreference,FRP,FRDpower);
            FRTreference=FRTvalues[0];
            FRDpower=FRTvalues[1];

            //Use our Controlloopmath class to find the power needed to go into our CRservo to achieve our desired target
            BLT.setPower(pidmath.PIDout(BLTreference,BLP,Kp,Kd,Ki,Kf,BLTtimer.seconds()));
            //BLD.setPower(BLDpower);
            BLTtimer.reset();

            BRT.setPower(pidmath.PIDout(BRTreference,BRP,Kp,Kd,Ki,Kf,BRTtimer.seconds()));
            //BRD.setPower(BRDpower);
            BRTtimer.reset();

            FLT.setPower(pidmath.PIDout(FLTreference,FLP,Kp,Kd,Ki,Kf,FLTtimer.seconds()));
            //FLD.setPower(FLDpower);
            FLTtimer.reset();

            FRT.setPower(pidmath.PIDout(FRTreference,FRP,Kp,Kd,Ki,Kf,FRTtimer.seconds()));
            //FRD.setPower(FRDpower);
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
