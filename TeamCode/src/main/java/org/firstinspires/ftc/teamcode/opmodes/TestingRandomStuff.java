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
    private CRServo INS = null;
    private Servo INFL = null, DROP = null, OTD = null;
    private DcMotorEx INE = null, OTE = null;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        INS = hardwareMap.get(CRServo.class, "INS");
        INFL = hardwareMap.get(Servo.class, "INFL");
        //INE = hardwareMap.get(DcMotorEx.class,"INE");

        //DROP = hardwareMap.get(Servo.class, "DROP");

        OTD = hardwareMap.get(Servo.class, "OTD");
        OTE = hardwareMap.get(DcMotorEx.class,"OTE");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        INS.setDirection(DcMotorSimple.Direction.REVERSE);

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {

            double INFP = 0;
            if (gamepad2.a){
                INFP = 0.975;
            }
            if (!gamepad2.a) {
                INFP = 0.2;
            }
            if (gamepad2.left_bumper) {
                INFP = 0.1;}
            INFL.setPosition(INFP);
            //DROP.setPosition(DROPP);

            double OTDP=0;
            if (gamepad2.y){
                OTDP = 1;
            }
            if (!gamepad2.y){
                OTDP = 0.5;
            }
            OTD.setPosition(OTDP);
            //depositing pos = 1
            //resting pos = 0.5
            //init pos = 0.1

            double OTEV;
            OTEV = gamepad2.right_stick_y;
            OTE.setPower(OTEV);

            double INEV;
            INEV = gamepad2.left_stick_y;
            //INE.setPower(INEV);

            if(INFP>0.2){
                INS.setPower(1);
            }
            else if(gamepad2.right_bumper){
                INS.setPower(1);
            }
            else if(gamepad2.b){
                INS.setPower(-0.25);

            }
            else{
                INS.setPower(0);}
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

}
