package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.maths.mathsOperations;


@Config
@TeleOp(name="TestingRandomStuff", group="Linear Opmode")
public class TestingRandomStuff extends LinearOpMode {


    FtcDashboard dashboard;
    private DcMotorEx m1 = null, m2 = null;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //INE = hardwareMap.get(DcMotorEx.class,"INE");

        //DROP = hardwareMap.get(Servo.class, "DROP");

        m1 = hardwareMap.get(DcMotorEx.class, "m1");
        m2 = hardwareMap.get(DcMotorEx.class,"m2");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {

            boolean simple = true;
            if(simple) {
                m1.setPower(gamepad1.left_stick_y);
                m2.setPower(gamepad1.right_stick_y);
            }
            else{
                double[] values = mathsOperations.diffyConvert(gamepad1.right_stick_x,gamepad1.left_stick_y);
                m1.setPower(values[0]);
                m2.setPower(values[1]);
            }

            String color = "#0f2259";
            String color1 = "#b28c00";
            telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
            telemetry.addData("", String.format("<span style=\"color:%s\">%s</span>",color1,"\n" +
                    "▬▬▬▬▬▬▬▬▬▬") + String.format("<span style=\"color:%s\">%s</span>",color,"" +"▬▬▬▬▬▬▬▬▬▬\n") +
                    String.format("<span style=\"color:%s\">%s</span>",color,"" +"░░░░░██╗██████╗░") + String.format("<span style=\"color:%s\">%s</span>",color1,"" +"|-----------------------------|\n") +
                    String.format("<span style=\"color:%s\">%s</span>",color,"" +"░░░░░██║██╔══██╗") + String.format("<span style=\"color:%s\">%s</span>",color1,"" +"|-----------------------------|\n") +
                    String.format("<span style=\"color:%s\">%s</span>",color,"" +"░░░░░██║██████╦╝") + String.format("<span style=\"color:%s\">%s</span>",color1,"" +"|------Jolly Blue--------|\n") +
                    String.format("<span style=\"color:%s\">%s</span>",color,"" +"██╗░░██║██╔══██╗") + String.format("<span style=\"color:%s\">%s</span>",color1,"" +"|------①⑨⑧②③-----|\n") +
                    String.format("<span style=\"color:%s\">%s</span>",color,"" +"╚█████╔╝██████╦╝") + String.format("<span style=\"color:%s\">%s</span>",color1,"" +"|--------Taylor------------|\n") +
                    String.format("<span style=\"color:%s\">%s</span>",color,"" +"░╚════╝░╚═════╝░") + String.format("<span style=\"color:%s\">%s</span>",color1,"" +"|--------Connor----------|\n") +
                    String.format("<span style=\"color:%s\">%s</span>",color1,"" +"▬▬▬▬▬▬▬▬▬▬") + String.format("<span style=\"color:%s\">%s</span>",color,""+"▬▬▬▬▬▬▬▬▬▬\n"));
            //telemetry.addData("TEST", String.format("<span style=\"color:%s\">%s</span>",color,gamepad1.right_stick_y));
            telemetry.update();

        }

    }

}
