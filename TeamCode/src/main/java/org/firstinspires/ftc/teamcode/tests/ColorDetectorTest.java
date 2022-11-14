package org.firstinspires.ftc.teamcode.tests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opencv.OpenCVDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


//@Disabled
@Autonomous(group = "Tests")
public class ColorDetectorTest extends LinearOpMode {

    OpenCvCamera camera;
    OpenCVDetectionPipeline.ColorDetectionPipeline pipeline;
    OpenCVDetectionPipeline.ColorDetectionPipeline.ColorPosition snapshotAnalysis = OpenCVDetectionPipeline.ColorDetectionPipeline.ColorPosition.ONE; // default

    public void runOpMode() throws InterruptedException{

        //INITIALIZATION ---------------------------------------------------------------------------
        int parkPosition = 1; //default value
        
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId); 
        pipeline = new OpenCVDetectionPipeline.ColorDetectionPipeline();
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        //LOOK FOR COLOR BEFORE CODE EXECUTES ------------------------------------------------------
        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        snapshotAnalysis = pipeline.getAnalysis();

        /*
         * Show that snapshot on the telemetry
         */
        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();

        //SAVE PARKING INFO ------------------------------------------------------------------------
        switch (snapshotAnalysis) //figure out where to park
        {
            case ONE:
            {
                parkPosition = 1;
                break;
            }

            case TWO:
            {
                parkPosition = 2;
                break;
            }

            case THREE:
            {
                parkPosition = 3;
                break;
            }
        }

        //EXECUTE PARKING CODE ---------------------------------------------------------------------
        switch (parkPosition)
        {
            case 1:
            {
                //PARKING CODE HERE
                break;
            }

            case 2:
            {
                //PARKING CODE HERE
                break;

            }

            case 3:
            {
                //PARKING CODE HERE
                break;
            }
        }
    }
}

