package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Hardware {
    //Create Motors
    public DcMotor backleftmotor = null;
    public DcMotor backrightmotor = null;
    public DcMotor swerver = null;
    public DcMotor test = null;

    public DcMotor backleftswerver = null;
    public DcMotor backrightswerver = null;
    public DcMotor frontleftswerver = null;
    public DcMotor frontrightswerver = null;

    //Create Servo
    public CRServo servo = null;

    public CRServo backleftservo = null;
    public CRServo backrightservo = null;
    public CRServo frontleftservo = null;
    public CRServo frontrightservo = null;

    //Additional Variables
    HardwareMap hardwaremap = null;
    public ElapsedTime runtime = new ElapsedTime();
    private Object HardwareMap;

    public Hardware(HardwareMap hwMap) {
        initialize(hwMap);
    }

    private void initialize(HardwareMap hwMap) {
        HardwareMap = hwMap;
        //Connect Motors
        backleftmotor = hardwaremap.get(DcMotor.class, "rightwheel");
        backrightmotor = hardwaremap.get(DcMotor.class, "leftwheel");
        swerver = hardwaremap.get(DcMotor.class, "swerver");
        test = hardwaremap.get(DcMotor.class, "test");

        backleftswerver = hardwaremap.get(DcMotor.class, "backleftswerver");
        backrightswerver = hardwaremap.get(DcMotor.class, "backrightswerver");
        frontleftswerver = hardwaremap.get(DcMotor.class, "frontleftswerver");
        frontrightswerver = hardwaremap.get(DcMotor.class, "frontrightswerver");

        //Connect Servo
        servo = hardwaremap.get(CRServo.class, "servo");

        backleftservo = hardwaremap.get(CRServo.class, "backleftservo");
        backrightservo = hardwaremap.get(CRServo.class, "backrightservo");
        frontleftservo = hardwaremap.get(CRServo.class, "frontleftservo");
        frontrightservo = hardwaremap.get(CRServo.class, "frontrightservo");

        //Set Up Motor Direction
        backrightmotor.setDirection(DcMotor.Direction.REVERSE);
        backleftmotor.setDirection(DcMotor.Direction.FORWARD);


        //Set Motor Mode
        backleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        swerver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set ZERO POWER BEHAVIOR
        backleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        swerver.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set Motors To Use No Power

    }
}
