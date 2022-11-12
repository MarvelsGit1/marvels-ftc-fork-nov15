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

    //VARIABLES
    public static int numCones = 5;
    public static int coneOffset = 150;
    public static int coneTop = 700;
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
        //HARDWARE INITIALIZATION ---------------------------------------------------------------
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        BaseRobotMethods robot = new BaseRobotMethods(hardwareMap);
        robot.telemetry = this.telemetry;
        robot.parent = this;

        robot.camservo.setPosition(0.15);
        robot.shoulder.setPosition(0.6);
        robot.grabber.setPosition(0.0);

        //ROAD RUNNER TRAJECTORIES -------------------------------------------------------------
        Pose2d startPose = new Pose2d(-33, -65, Math.toRadians(180)); //starting position
        drive.setPoseEstimate(startPose);

        //PRELOAD SCORE
        TrajectorySequence conePreload = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-14, -65)) //backup from starting position
                .addTemporalMarker(0, () -> robot.liftArm(3450, 1)) //raise arm
                .strafeTo(new Vector2d(-14, -13)) //go to middle of field
                .lineTo(new Vector2d(-18.25, -13)) //adjust a bit forward to score preload
                .addTemporalMarker(4, () -> robot.shoulder.setPosition(0.95)) //stretch arm out
                .addTemporalMarker(6, () -> robot.grabber.setPosition(1.0)) //release cone
                .addTemporalMarker(7, () -> robot.shoulder.setPosition(0.6)) //bring arm back
                .addTemporalMarker(8, () -> robot.liftArm(coneTop, 1)) //drop arm to top of cone stack
                .build();

        //GOTO CONE STACK + PINCH CONE
        TrajectorySequence coneStack = drive.trajectorySequenceBuilder(conePreload.end())
                .lineTo(new Vector2d(-60, -13)) //cone stack pos
                .addTemporalMarker(4, () -> robot.grabber.setPosition(0.0)) //pinch cone
                .build();

        //BACK UP FROM CONE STACK AND SCORE
        TrajectorySequence coneScore = drive.trajectorySequenceBuilder(coneStack.end())
                .addTemporalMarker(0, () -> robot.liftArm(3450, 1)) //raise arm
                .lineTo(new Vector2d(-18.25, -13)) //goto score pos
                .addTemporalMarker(2, () -> robot.shoulder.setPosition(0.95)) //stretch arm out
                .addTemporalMarker(6, () -> robot.grabber.setPosition(1.0)) //release cone
                .addTemporalMarker(7, () -> robot.shoulder.setPosition(0.6)) //bring arm back
                .build();

        //PARKING 1
        TrajectorySequence parkingOne = drive.trajectorySequenceBuilder(coneScore.end())
                .lineTo(new Vector2d(-14, -65))
                .strafeTo(new Vector2d(-14, -13))
                .build();

        //PARKING 2
        TrajectorySequence parkingTwo = drive.trajectorySequenceBuilder(coneScore.end())
                .lineTo(new Vector2d(-14, -65))
                .strafeTo(new Vector2d(-14, -13))
                .build();

        //PARKING 3
        TrajectorySequence parkingThree = drive.trajectorySequenceBuilder(coneScore.end())
                .lineTo(new Vector2d(-14, -65))
                .strafeTo(new Vector2d(-14, -13))
                .build();

        //CAMERA INITIALIZATION ----------------------------------------------------------------
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        //WHILE WAIT FOR START FIND APRIL TAG
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

        //EXECUTE SCRIPT ------------------------------------------------------------------
        drive.followTrajectorySequence(conePreload);

        //LOOP CONE PICKUP
        for (int i = 0; i < numCones; i++) {
            coneHeight = coneTop - i*coneOffset;
            robot.liftArm(coneHeight, 1);
            drive.followTrajectorySequenceAsync(coneStack);
            drive.followTrajectorySequence(coneScore);
        }

        //PARK
        sleep(15000);
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