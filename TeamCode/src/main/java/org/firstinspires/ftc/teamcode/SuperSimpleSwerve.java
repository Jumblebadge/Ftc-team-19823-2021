package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="SuperSimpleSwerve", group="Linear Opmode")
public class SuperSimpleSwerve extends LinearOpMode {


    private CRServo frontleftservo = null;
    private DcMotor frontleftswerver = null;

    //@Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        frontleftservo = hardwareMap.get(CRServo.class, "frontleftservo");
        frontleftswerver = hardwareMap.get(DcMotor.class, "frontleftswerver");




        waitForStart();

        while (opModeIsActive()) {


           double frontleftservopower;
           frontleftservopower = gamepad1.right_stick_x;


           double frontleftswerverpower;
           double frontleftservodiv;
           double frontleftmotorclip;
           double frontleftadd;


           frontleftmotorclip = gamepad1.left_stick_y;
           frontleftservodiv = frontleftservopower/6*4/6;
           frontleftmotorclip = Range.clip(frontleftmotorclip,-0.83, 0.83);
           frontleftadd = frontleftmotorclip+frontleftservodiv;
           frontleftswerverpower = frontleftadd;


            frontleftservo.setPower(frontleftservopower*-1);
            frontleftswerver.setPower(frontleftswerverpower*-1);




            float a = frontleftswerver.getCurrentPosition();
            telemetry.addData("Current Position",a);
            telemetry.update();

        }
    }

}
