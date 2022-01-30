 package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="SimpleSwerve", group="Linear Opmode")
public class SimpleSwerve extends LinearOpMode {

    private DcMotor frontrightswerver = null;
    private CRServo frontrightservo = null;
    private CRServo frontleftservo = null;
    private DcMotor frontleftswerver = null;
    private DcMotor backrightswerver = null;
    private CRServo backrightservo = null;
    private CRServo backleftservo = null;
    private DcMotor backleftswerver = null;
    //private DcMotor corehextest = null;
    //n private DcMotor armUpDown = null;
    //double toggle_count;
    //boolean directionState = false;


    //@Override
    public void runOpMode() {
        //telemetry.addData("Status", "Initialized");
        //telemetry.update();

        //toggle_count = 0;
        frontrightswerver = hardwareMap.get(DcMotor.class, "frontrightswerver");
        frontrightservo = hardwareMap.get(CRServo.class, "frontrightservo");
        frontleftservo = hardwareMap.get(CRServo.class, "frontleftservo");
        frontleftswerver = hardwareMap.get(DcMotor.class, "frontleftswerver");
        backrightswerver = hardwareMap.get(DcMotor.class, "backrightswerver");
        backrightservo = hardwareMap.get(CRServo.class, "backrightservo");
        backleftservo = hardwareMap.get(CRServo.class, "backleftservo");
        backleftswerver = hardwareMap.get(DcMotor.class, "backleftswerver");
        //corehextest = hardwareMap.get(DcMotor.class,"corehextest");
        //armUpDown = hardwareMap.get(DcMotor.class, "armUpDown");
        //left = hardwareMap.get(DcMotor.class, "left");
        //right = hardwareMap.get(DcMotor.class, "right");


        waitForStart();


        while (opModeIsActive()) {


            // if(directionState == false){
            //   if(gamepad1.x){
            //     toggle_count += 1;
            //   directionState = true;

            // armUpDown.setPower(1);

            //}else{
            //  if(gamepad1.a){
            //    toggle_count += 1;
            //  directionState = false;

            //armUpDown.setPower(-1);
            //}
            //}
            //boolean directionState = false;
            //toggle_count = 0;


            //telemetry.addData("Direction State:", directionState);
            //telemetry.addData("ArmUpDown:", toggle_count);

            double frontleftservopower;
            frontleftservopower = gamepad1.right_stick_x;


            double frontleftswerverpower;
            double frontleftservodiv;
            double frontleftmotorclip;
            double frontleftadded;


            frontleftmotorclip = gamepad1.left_stick_y;
            frontleftservodiv = frontleftservopower / 6 * 4 / 6;
            frontleftmotorclip = Range.clip(frontleftmotorclip, -0.83, 0.83);
            frontleftadded = frontleftmotorclip + frontleftservodiv;
            frontleftswerverpower = frontleftadded;


            frontleftservo.setPower(frontleftservopower * -1);
            frontleftswerver.setPower(frontleftswerverpower * -1);

            frontrightservo.setPower(frontleftservopower * -1);
            frontrightswerver.setPower(frontleftswerverpower * -1);

            backleftservo.setPower(frontleftservopower * -1);
            backleftswerver.setPower(frontleftswerverpower * -1);

            backrightservo.setPower(frontleftservopower * -1);
            backrightswerver.setPower(frontleftswerverpower * -1);

            //armUpDown.setPower(gamepad1.right_stick_y);


            telemetry.addData("e",frontleftswerver.getCurrentPosition());
            telemetry.update();

        }
    }

}


