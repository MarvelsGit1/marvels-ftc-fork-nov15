package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.dashboard.config.Config;

@Disabled
@Config
@Autonomous(group = "Tests")
public class HardwareTest extends LinearOpMode {

    private DcMotor frontright;
    private DcMotor backright;
    private DcMotor frontleft;
    private DcMotor backleft;
    private DcMotor arm;

    public static Servo grabber;
    public static Servo shoulder;
    public static Servo camservo;


    private String state = "grabberPos";
    private Boolean translating = false;
    private Boolean firstTouch = false;
    private int destination;
    private double position = 0.1;

    public ElapsedTime timer;

    public void runOpMode() {
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        arm = hardwareMap.get(DcMotor.class, "arm");

        camservo=hardwareMap.get(Servo.class,"camservo");
        grabber=hardwareMap.get(Servo.class,"grabber");
        shoulder=hardwareMap.get(Servo.class,"shoulder");

        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backright.setDirection(DcMotor.Direction.REVERSE);
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()){

            //arm = bottom [0 - 3300] top
            camservo.setPosition(0.85); // left side [0.15 - 0.85] right side
            shoulder.setPosition(0.6); // in [0.6 - 0.8X (mid) - 1.0] out
            grabber.setPosition(1.0); // in [0.0 - 1.0] out
        }
    }
}