package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.I2cDevice;



@TeleOp(name="swervy", group="Linear Opmode")
public class swervy extends LinearOpMode {


    private CRServo BLT;
    private AnalogInput BLE;
    private AnalogInput BRE;
    private AnalogInput FLE;
    private AnalogInput FRE;

    double BLP = 0;
    double BRP = 0;
    double FLP = 0;
    double FRP = 0;

    public void runOpMode() {
        ElapsedTime timer =  new ElapsedTime();

        BLE = hardwareMap.get(AnalogInput.class, "BLE");
        BRE = hardwareMap.get(AnalogInput.class, "BRE");
        FLE = hardwareMap.get(AnalogInput.class, "FLE");
        FRE = hardwareMap.get(AnalogInput.class, "FRE");

        BLT = hardwareMap.get(CRServo.class,"BLT");


        waitForStart();


        while (opModeIsActive()) {

            double x1 = gamepad1.left_stick_x;
            double y1 = gamepad1.left_stick_y;
            double x2 = gamepad1.right_stick_x;
            double y2 = gamepad1.right_stick_y;

            BLP = BLE.getVoltage() * 74.16;
            BRP = BRE.getVoltage() * 74.16;
            FLP = FLE.getVoltage() * 74.16;
            FRP = FRE.getVoltage() * 74.16;
            BLT.setPower(1);

            BLP = BLP%360 - (BLP%360 > 180 ? 360 : 0);
            BRP = BRP%360 - (BRP%360 > 180 ? 360 : 0);
            FLP = FLP%360 - (FLP%360 > 180 ? 360 : 0);
            FRP = FRP%360 - (FRP%360 > 180 ? 360 : 0);

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

            telemetry.addData("BLP",BLP);
            telemetry.addData("BRP",BRP);
            telemetry.addData("FLP",FLP);
            telemetry.addData("FRP",FRP);
            telemetry.update();
        }}
    }


