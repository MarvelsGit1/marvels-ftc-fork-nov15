package org.firstinspires.ftc.teamcode;

//CAMERA IMPORTS
import android.annotation.SuppressLint;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opencv.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import java.util.ArrayList;

//ROAD RUNNER IMPORTS
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "Red Autos")
public class RedLeftAutonomous extends LinearOpMode {

    //VARIABLES ------------------------------------------------------------------------------------
    public static int poleHeight = 3465; //3475 prior
    public static int numCones = 5;
    public static int coneOffset = 140;
    public static int coneTop = 550; //350 prior
    public static int coneHeight;

    //CAMERA
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException{
        //HARDWARE INITIALIZATION ------------------------------------------------------------------
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        BaseRobotMethods robot = new BaseRobotMethods(hardwareMap);
        robot.telemetry = this.telemetry;
        robot.parent = this;

        robot.camservo.setPosition(0.07);
        robot.shoulder.setPosition(0.6);
        robot.grabber.setPosition(0.0);

        //ROAD RUNNER TRAJECTORIES -----------------------------------------------------------------
        Pose2d startPose = new Pose2d(-33, -65, Math.toRadians(180)); //starting position
        drive.setPoseEstimate(startPose);

        //PRELOAD SCORE
        TrajectorySequence conePreload = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-14, -65)) //backup from starting position
                .strafeTo(new Vector2d(-14, -16.5)) //go to middle of field
                .lineTo(new Vector2d(-16.75, -16.25)) //adjust a bit forward to score preload
                //5 seconds
                .waitSeconds(2.5)
                .addTemporalMarker(0, () -> robot.liftArm(poleHeight, 1)) //raise arm
                .addTemporalMarker(4, () -> robot.shoulder.setPosition(0.95)) //stretch arm out
                .addTemporalMarker(5.5, () -> robot.grabber.setPosition(1.0)) //release cone
                .addTemporalMarker(6.5, () -> robot.shoulder.setPosition(0.6)) //bring arm back
                .addTemporalMarker(7, () -> robot.liftArm(coneTop, 1)) //drop arm to top of cone stack
                .build();

        //GOTO CONE STACK + PINCH CONE
        TrajectorySequence coneStack = drive.trajectorySequenceBuilder(conePreload.end())
                .lineTo(new Vector2d(-65, -14.5)) //cone stack pos
                //2.5 seconds
                .waitSeconds(1)
                .addTemporalMarker(2.5, () -> robot.grabber.setPosition(0.0)) //pinch cone
                .build();

        //BACK UP FROM CONE STACK AND SCORE
        TrajectorySequence coneScore = drive.trajectorySequenceBuilder(coneStack.end())
                .lineTo(new Vector2d(-16.75, -16.5)) //goto score pos
                .waitSeconds(5)
                .addTemporalMarker(0, () -> robot.liftArm(poleHeight, 1)) //raise arm
                .addTemporalMarker(4, () -> robot.shoulder.setPosition(0.95)) //stretch arm out
                .addTemporalMarker(5.5, () -> robot.grabber.setPosition(1.0)) //release cone
                .addTemporalMarker(6.5, () -> robot.shoulder.setPosition(0.6)) //bring arm back
                .addTemporalMarker(7, () -> robot.liftArm(coneTop, 1)) //drop arm to top of cone stack
                .build();

        //PARKING LEFT
        TrajectorySequence parkingOne = drive.trajectorySequenceBuilder(coneScore.end())
                .lineTo(new Vector2d(-62, -14.5))
                .build();

        //PARKING MIDDLE
        TrajectorySequence parkingTwo = drive.trajectorySequenceBuilder(coneScore.end())
                .lineTo(new Vector2d(-40, -14.5))
                .build();

        //PARKING RIGHT
        TrajectorySequence parkingThree = drive.trajectorySequenceBuilder(coneScore.end())
                .lineTo(new Vector2d(-14, -14.5))
                .build();

        //CAMERA INITIALIZATION --------------------------------------------------------------------
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        //WHILE WAIT FOR START FIND APRIL TAG ------------------------------------------------------
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;
                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }

            telemetry.update();
            sleep(20);
        }

        //EXECUTE SCRIPT ---------------------------------------------------------------------------
        //PRELOAD
        drive.followTrajectorySequence(conePreload);
        drive.followTrajectorySequence(coneStack);
        drive.followTrajectorySequence(coneScore);

        //LOOP CONE PICKUP
//        for (int i = 0; i < numCones; i++) {
//            coneHeight = coneTop - i*coneOffset;
//            robot.liftArm(coneHeight, 1);
//            drive.followTrajectorySequence(coneStack);
//            drive.followTrajectorySequence(coneScore);
//        }

        //PARK
        if(tagOfInterest == null || tagOfInterest.id == RIGHT){
             //drive.followTrajectorySequence(coneStack);
            drive.followTrajectorySequence(parkingThree);
        }else if(tagOfInterest.id == MIDDLE){
            drive.followTrajectorySequence(parkingTwo);
        }else{
            //already in right spot
            drive.followTrajectorySequence(parkingOne);
        }
    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}