package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;




@TeleOp(name="ControllerTesting", group="Linear Opmode")
public class ControllerTesting extends LinearOpMode {

    double BLTreference = 0;
    double BRTreference = 0;
    double FLTreference = 0;
    double FRTreference = 0;

    BNO055IMU IMU;


    public void runOpMode() {
        waitForStart();


        while (opModeIsActive()) {



            double x1 = gamepad1.left_stick_x;
            double y1 = -gamepad1.left_stick_y;
            double x2 = gamepad1.right_stick_x;
            double y2 = gamepad1.right_stick_y;

            if (y1 == 0){
                y1 = 1;
            }

            double atan = Math.atan2(x1,y1);
            atan *= 57.2958;

            BLTreference = atan;
            BRTreference = atan;
            FLTreference = atan;
            FRTreference = atan;



            telemetry.addData("x1",x1);
            telemetry.addData("y1",y1);
            telemetry.addData("x2",x2);
            telemetry.addData("y2",y2);
            telemetry.addData("FRTR",FRTreference);
            telemetry.addData("FLTR",FLTreference);
            telemetry.addData("BRTR",BRTreference);
            telemetry.addData("BLTR",BLTreference);
            telemetry.update();

        }
    }
}


