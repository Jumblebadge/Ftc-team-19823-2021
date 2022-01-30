package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="FirstSwerve", group="Linear Opmode")
public class FirstSwerve extends LinearOpMode {

    private DcMotorEx BLD = null;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        BLD = hardwareMap.get(DcMotorEx.class, "BLD");
        BLD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        waitForStart();

        //13.25inL   14.25inW



        while (opModeIsActive()) {


            double r = Math.sqrt ((13.25 * 13.25) + (14.25 * 14.25));
            gamepad1.left_stick_y *= -1;

            double a = gamepad1.left_stick_x - gamepad1.right_stick_x * (13.25 / r);
            double b = gamepad1.left_stick_x + gamepad1.right_stick_x * (13.25 / r);
            double c = gamepad1.left_stick_y - gamepad1.right_stick_x * (14.25 / r);
            double d = gamepad1.left_stick_y + gamepad1.right_stick_x * (14.25 / r);

            double backRightSpeed1 = Math.sqrt ((a * a) + (d * d));
            double backLeftSpeed1 = Math.sqrt ((a * a) + (c * c));
            double frontRightSpeed1 = Math.sqrt ((b * b) + (d * d));
            double frontLeftSpeed1 = Math.sqrt ((b * b) + (c * c));

            double backRightAngle1 = Math.atan2 (a, d) / Math.PI;
            double backLeftAngle1 = Math.atan2 (a, c) / Math.PI;
            double frontRightAngle1 = Math.atan2 (b, d) / Math.PI;
            double frontLeftAngle1 = Math.atan2 (b, c) / Math.PI;

            double backRightAngle = 0;
            double backLeftAngle = 0;
            double frontRightAngle = 0;
            double frontLeftAngle = 0;


            double backRightSpeed = backRightSpeed1;
            double backLeftSpeed = backLeftSpeed1;
            double frontRightSpeed = frontRightSpeed1;
            double frontLeftSpeed = frontLeftSpeed1;

            backRightAngle = (backRightAngle1)*180;
            backLeftAngle = (backLeftAngle1)*180;
            frontRightAngle = frontRightAngle1*180;
            frontLeftAngle = frontLeftAngle1*180;

            double enccurpos = BLD.getCurrentPosition();
            double curposdeg = enccurpos*11.25;

            if (enccurpos >= 32) {

                enccurpos = 0;
                curposdeg = 0;
                BLD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                //telemetry.update();

            }
            else if (enccurpos <= -32){
                enccurpos = 0;
                curposdeg = 0;
                BLD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            }

            telemetry.addData("backrightspeed",backRightSpeed);
            telemetry.addData("backleftspeeed",backLeftSpeed);
            telemetry.addData("frontrightspeed",frontRightSpeed);
            telemetry.addData("frontleftspeed",frontLeftSpeed);

            telemetry.addData("backrightangle",backRightAngle);
            telemetry.addData("backleftangle",backLeftAngle);
            telemetry.addData("frontrightangle",frontRightAngle);
            telemetry.addData("frontleftangle",frontLeftAngle);


            telemetry.addData("enccurpos",enccurpos);
            telemetry.addData("curpodeg",curposdeg);

            telemetry.update();
            

        }



        }
    }


