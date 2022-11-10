package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous
public class BlueLeftAutonomous extends LinearOpMode {

//    OpenCvInternalCamera phoneCam;
//    BlueLeftColorDetector.ColorDetectionPipeline pipeline;
//    BlueLeftColorDetector.ColorDetectionPipeline.ColorPosition snapshotAnalysis = BlueLeftColorDetector.ColorDetectionPipeline.ColorPosition.ONE; // default

    public int parkPosition;
    private Servo camservo;

    public DistanceSensor dsensor;
    private Servo grabber;
    public int errorColorFront;
    public int lastErrorColorFront;
    public int goalFront;
    public int adjustFront;
    public double kPFront;
    public double kDFront;;

    public int errorColorBack;
    public int lastErrorColorBack;
    public int goalBack;
    public int adjustBack;
    public double kPBack;
    public double kDBack;

    public int defaultVel;
    public ElapsedTime runtime = new ElapsedTime();

    public boolean bothAligned = false;
    public boolean coneGrabbed = false;
    public boolean ready = false;
    public DcMotorEx frontright;
    public DcMotorEx frontleft;
    public DcMotorEx backright;
    public DcMotorEx backleft;
    public DcMotorEx arm;

    public int goalDist;

    ColorSensor csf;
    ColorSensor csb;
    ColorSensor csl;
    ColorSensor csr;

    public void runOpMode() throws InterruptedException{

//        camservo=hardwareMap.get(Servo.class,"camservo");
//        camservo.setPosition(0.15);
//
//        grabber=hardwareMap.get(Servo.class,"grabber");
//        grabber.setPosition(0.1);
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        // pipeline = new BlueLeftColorDetector.ColorDetectionPipeline();
        // phoneCam.setPipeline(pipeline);

        // phoneC   am.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        // {
        //     @Override
        //     public void onOpened()
        //     {
        //         phoneCam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
        //     }

        //     @Override
        //     public void onError(int errorCode) {}
        // });

        // while (!isStarted() && !isStopRequested())
        // {
        //     telemetry.addData("Realtime analysis", pipeline.getAnalysis());
        //     telemetry.update();

        //     // Don't burn CPU cycles busy-looping in this sample
        //     sleep(50);
        // }

        // snapshotAnalysis = pipeline.getAnalysis();

        // /*
        //  * Show that snapshot on the telemetry
        //  */
        // telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        // telemetry.update();

        // switch (snapshotAnalysis) //figure out where to park
        // {
        //     case ONE:
        //     {
        //         parkPosition = 1;
        //         break;
        //     }

        //     case TWO:
        //     {
        //         parkPosition = 2;
        //         break;
        //     }

        //     case THREE:
        //     {
        //         parkPosition = 3;
        //         break;
        //     }
        // }


        BaseMotorMethods robot = new BaseMotorMethods(hardwareMap);
        robot.telemetry = this.telemetry;
        robot.parent = this;

        waitForStart();
        robot.moveForward(0.5,500);// one tile = 480 Ratio 1:480
        sleep(1000);
        robot.moveBackward(0.5,500);
        // BM.moveRight(0.5,480);
        // BM.stopMovement();



        //     //scores first preloaded cone

        //     moveBackward(0.5,150);
        //     sleep(200);

        //     for (int i = 0; i < 0; i++) {

        //         moveRight(0.5, 800);
        //         sleep(400);
        //         moveForward(0.5,1600);
        //         moveArm(800);

        //   defaultVel = 300;

        //     kPFront = 8; // 1/10th desired velocity
        //     kPBack = 7;
        //     kDFront = 2;
        //     kDBack = 2;

        //     goalFront = 280; //Value Requires Calibration. Put Higher Than Desired Value +50
        //     goalBack = 280;
        //     goalDist = 10; //CM
        //     lastErrorColorFront = 0;
        //     lastErrorColorBack = 0;

        //     while(dsensor.getDistance(DistanceUnit.CM) > goalDist && dsensor.getDistance(DistanceUnit.CM) < 50000){

        //         errorColorFront = goalFront - csf.blue();
        //         errorColorBack = goalBack - csb.blue();

        //         adjustFront = (int) (kPFront*(float)errorColorFront + kDFront*(float)(errorColorFront - lastErrorColorFront));
        //         adjustBack = (int) (kPBack*(float)errorColorBack + kDBack*(float)(errorColorBack - lastErrorColorBack));

        //         if(csr.blue() > 270){
        //             frontleft.setVelocity(-defaultVel - adjustFront + adjustBack); //forward + cw rot + ccw rot
        //             frontright.setVelocity(-defaultVel + adjustFront - adjustBack);
        //             backleft.setVelocity(defaultVel + adjustFront - adjustBack);
        //             backright.setVelocity(defaultVel - adjustFront + adjustBack);
        //         }

        //         else {
        //         frontleft.setVelocity(-defaultVel + adjustFront - adjustBack); //forward + ccw rot + cw rot
        //         frontright.setVelocity(-defaultVel - adjustFront + adjustBack);
        //         backleft.setVelocity(defaultVel - adjustFront + adjustBack);
        //         backright.setVelocity(defaultVel + adjustFront - adjustBack);

        //         lastErrorColorFront = errorColorFront; //store for kD later
        //         lastErrorColorFront = errorColorBack;
        //         }
        //     }

        //         frontright.setVelocity(0);
        //         frontleft.setVelocity(0);
        //         backright.setVelocity(0);
        //         backleft.setVelocity(0);

        //         moveLeft(0.7, 60);

        //         moveArm(400 - i*150); //THIS NEEDS TO BE VARIABLE
        //         sleep(900);

        //         grabber.setPosition(0.55);
        //         sleep(300);
        //         moveArm(2500);
        //         sleep(600);
        //         // picks up a cone
        //         moveBackward(0.5, 2300);
        //         sleep(600);
        //         moveLeft(0.5, 590);
        //         sleep(200);


        //         telemetry.addLine("Finished Move Right");
        //         telemetry.update();
        //         sleep(500);
        //         // 1st cycle
        //         telemetry.addLine("slept");
        //         telemetry.update();
        //         sleep(500);
        //         stopMovement();

        //         telemetry.addData("distance:", dsensor.getDistance(DistanceUnit.CM));
        //         telemetry.update();
        //     }

        //     switch (parkPosition)
        //     {
        //         case 1:
        //         {
        //             moveLeft(0.5,650);
        //             sleep(400);
        //             turnRight(0.5, 50);
        //             sleep(400);
        //             moveForward(0.3, 2150);
        //             sleep(400);
        //             break;
        //         }

        //         case 2:
        //         {
        //             moveLeft(0.5,650);
        //             sleep(400);
        //             turnRight(0.5, 50);
        //             sleep(400);
        //             moveForward(0.3, 1050);
        //             sleep(400);
        //             break;

        //         }

        //         case 3:
        //         {
        //             moveLeft(0.5,650);
        //             sleep(400);
        //             break;
        //         }
        //     }

    }
}
