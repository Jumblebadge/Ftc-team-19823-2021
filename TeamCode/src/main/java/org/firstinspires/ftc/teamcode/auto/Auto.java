package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous public class Auto extends LinearOpMode {


    public static double Kp = 0.2;
    public static double tolerance = 0;
    public static double Ki = 0;
    public static double Kd = 0.0001;
    public static double Kf = 0;

    public static double BLPC = 15;
    public static double FRPC = 5;
    public static double BRPC = -3;
    public static double FLPC = 2;

    double BLTintegralSum = 0;
    double BLTlastError = 0;
    double BLTerror = 0;


    double BRTintegralSum = 0;
    double BRTlastError = 0;
    double BRTerror = 0;


    double FLTintegralSum = 0;
    double FLTlastError = 0;
    double FLTerror = 0;


    double FRTintegralSum = 0;
    double FRTlastError = 0;
    double FRTerror = 0;

    private AnalogInput BLE = null;
    private AnalogInput BRE = null;
    private AnalogInput FLE = null;
    private AnalogInput FRE = null;

    private CRServo BLT = null;
    private CRServo BRT = null;
    private CRServo FLT = null;
    private CRServo FRT = null;

    double BLP = 0;
    double BRP = 0;
    double FLP = 0;
    double FRP = 0;

    private DcMotorEx BLD = null;
    private DcMotorEx BRD = null;
    private DcMotorEx FLD = null;
    private DcMotorEx FRD = null;
    static final double HD_COUNTS_PER_REV = 560;
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double WHEEL_CIRCUMFERENCE_MM = 273;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    static final double DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;

    private ElapsedTime runtime = new ElapsedTime();

    ElapsedTime BLTtimer =  new ElapsedTime();
    ElapsedTime BRTtimer =  new ElapsedTime();
    ElapsedTime FLTtimer =  new ElapsedTime();
    ElapsedTime FRTtimer =  new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

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

        waitForStart();
        if (opModeIsActive()) {

        }
    }
    private void drive(double power, double BLTInches, double BRTInches, double FLTInches, double FRTInches, double BLTreference, double BRTreference, double FLTreference, double FRTreference) {

        int BLTarget;
        int BRTarget;
        int FLTarget;
        int FRTarget;


        if (opModeIsActive()) {

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

            FRTreference -= FRPC;
            FLTreference -= FLPC;
            BRTreference -= BRPC;
            BLTreference -= BLPC;

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

            // Create target positions
            BLTarget = BLD.getCurrentPosition() + (int)(BLTInches * DRIVE_COUNTS_PER_IN);
            BRTarget = BRD.getCurrentPosition() + (int)(BRTInches * DRIVE_COUNTS_PER_IN);
            FLTarget = FLD.getCurrentPosition() + (int)(FLTInches * DRIVE_COUNTS_PER_IN);
            FRTarget = FRD.getCurrentPosition() + (int)(FRTInches * DRIVE_COUNTS_PER_IN);

            // set target position
            BLD.setTargetPosition(BLTarget);
            BRD.setTargetPosition(BRTarget);
            FLD.setTargetPosition(FLTarget);
            FRD.setTargetPosition(FRTarget);

            //switch to run to position mode
            BLD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FLD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRD.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //run to position at the desiginated power
            BLD.setPower(power);
            BRD.setPower(power);
            FLD.setPower(power);
            FRD.setPower(power);

            // wait until both motors are no longer busy running to position
            while (opModeIsActive() && (BLD.isBusy() || BRD.isBusy() || FLD.isBusy() || FRD.isBusy())) {
            }

            // set motor power back to 0
            BLD.setPower(0);
            BRD.setPower(0);
            FLD.setPower(0);
            FRD.setPower(0);

        }
    }
}
