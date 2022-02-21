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


@Config
@TeleOp(name="TestingRandomStuff", group="Linear Opmode")
public class TestingRandomStuff extends LinearOpMode {


    FtcDashboard dashboard;
    private DcMotorEx OTE = null;
    private Servo OTD = null;


    BNO055IMU IMU;



    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        OTE = hardwareMap.get(DcMotorEx.class,"OTE");
        IMU = hardwareMap.get(BNO055IMU.class, "IMU");
        OTD = hardwareMap.get(Servo.class,"OTD");

        IMU.initialize(parameters);
        IMU.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        dashboard = FtcDashboard.getInstance();


        waitForStart();

        while (opModeIsActive()) {
            OTE.setPower(gamepad1.right_stick_y);
            double OTDP=0;
            if (gamepad2.y){
                OTDP = 1;
            }
            if (!gamepad2.y){
                OTDP = 0.5;
            }
            OTD.setPosition(OTDP);
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
            telemetry.addData("TEST", String.format("<span style=\"color:%s\">%s</span>",color,gamepad1.right_stick_y));
            telemetry.update();





        }

    }
    private void ANGELWRP(double angle){
        if(angle <= -180) {
            angle += 360;
        }
        if(angle > 180) {
            angle -= 360;
        }
    }
}
