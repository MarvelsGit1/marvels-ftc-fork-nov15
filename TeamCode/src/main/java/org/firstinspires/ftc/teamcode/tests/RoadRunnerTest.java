package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous(group = "Tests")
public class RoadRunnerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-33, -68, Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(2)
                .lineTo(new Vector2d(-13, -68))
                //.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> servo.setPosition(0)) // Lower servo
                .waitSeconds(1)
                //.UNSTABLE_addTemporalMarkerOffset(-0.3, () -> servo.setPosition(1)) // Raise servo
                .strafeTo(new Vector2d(-13, -16
                ))
                .build();

        waitForStart();

        drive.followTrajectorySequence(trajSeq);
    }
}