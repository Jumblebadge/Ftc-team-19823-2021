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



@TeleOp(name="ControllerTesting", group="Linear Opmode")
public class ControllerTesting extends LinearOpMode {



    public void runOpMode() {
        waitForStart();


        while (opModeIsActive()) {

            final double L = 14.25;
            final double W = 14.25;

            double r = Math.sqrt((L * L) + (W * W));
            gamepad1.left_stick_y *= -1;

            double a = gamepad1.left_stick_x - gamepad1.right_stick_x * (L / r);
            double b = gamepad1.left_stick_x + gamepad1.right_stick_x * (L / r);
            double c = gamepad1.left_stick_y - gamepad1.right_stick_x * (W / r);
            double d = gamepad1.left_stick_y + gamepad1.right_stick_x * (W / r);

            double backRightSpeed1 = Math.sqrt((a * a) + (d * d));
            double backLeftSpeed1 = Math.sqrt((a * a) + (c * c));
            double frontRightSpeed1 = Math.sqrt((b * b) + (d * d));
            double frontLeftSpeed1 = Math.sqrt((b * b) + (c * c));

            double backRightAngle1 = Math.atan2(a, d);
            double backLeftAngle1 = Math.atan2(a, c);
            double frontRightAngle1 = Math.atan2(b, d);
            double frontLeftAngle1 = Math.atan2(b, c);

            double backRightSpeed = backRightSpeed1;
            double backLeftSpeed = backLeftSpeed1;
            double frontRightSpeed = frontRightSpeed1;
            double frontLeftSpeed = frontLeftSpeed1;

            double backRightAngle = backRightAngle1 * 180;
            double backLeftAngle = backLeftAngle1 * 180;
            double frontRightAngle = frontRightAngle1 * 180;
            double frontLeftAngle = frontLeftAngle1 * 180;


            double x1 = gamepad1.left_stick_x;
            double y1 = gamepad1.left_stick_y;
            double x2 = gamepad1.right_stick_x;
            double y2 = gamepad1.right_stick_y;
            telemetry.addData("x1",x1);
            telemetry.addData("y1",y1);
            telemetry.addData("x2",x2);
            telemetry.addData("y2",y2);
            telemetry.addData("BRA",backRightAngle1);
            telemetry.addData("BLA",backLeftAngle1);
            telemetry.addData("FRA",frontRightAngle1);
            telemetry.addData("FLA",frontLeftAngle1);
            telemetry.update();

        }
    }
}


