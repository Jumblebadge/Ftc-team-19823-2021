package org.firstinspires.ftc.teamcode.opmodes;

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
import org.firstinspires.ftc.teamcode.maths.mathsOperations;
import org.firstinspires.ftc.teamcode.maths.swerveMaths;
import org.firstinspires.ftc.teamcode.maths.PIDmaths;


@TeleOp(name="Encodertuning", group="Linear Opmode")
public class Encodertuning extends LinearOpMode {


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
        swerveMaths swavemath = new swerveMaths();


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

            BLP=mathsOperations.angleWrap(BLP);
            BRP=mathsOperations.angleWrap(BRP);
            FLP=mathsOperations.angleWrap(FLP);
            FRP=mathsOperations.angleWrap(FRP);

            double atan = Math.atan2(-x1,-y1);
            atan *= 57.2958;


            telemetry.addData("BLP",BLP);
            telemetry.addData("BRP",BRP);
            telemetry.addData("FLP",FLP);
            telemetry.addData("FRP",FRP);
            telemetry.addData("ATAN",atan);
            telemetry.update();
        }}
    }


