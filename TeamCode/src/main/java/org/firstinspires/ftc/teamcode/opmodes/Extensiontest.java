package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.AnalogSensor;

@TeleOp(name="Extensiontest", group="Linear Opmode")
public class Extensiontest extends LinearOpMode {

    private DcMotorEx Intakeextension;
    private Servo lift;

    public void runOpMode() throws InterruptedException {





        Intakeextension = hardwareMap.get(DcMotorEx.class, "IE");
        lift = hardwareMap.get(Servo.class,"lift");

        waitForStart();




        while (opModeIsActive()) {

            double x1= gamepad1.left_stick_x;
            double y1= gamepad1.left_stick_y;
            double x2= gamepad1.right_stick_x;

            Intakeextension.setPower(gamepad1.left_stick_y/2);
            while (gamepad1.a=true){


            }


            telemetry.addData("Leftx",x1);
            telemetry.addData("Lefty",y1);
            telemetry.addData("rightx",x2);
            telemetry.update();
        }
    }
}