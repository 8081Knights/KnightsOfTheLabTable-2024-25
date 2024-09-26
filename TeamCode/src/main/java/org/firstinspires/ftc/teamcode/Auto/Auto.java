package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareSoftware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name="Auto")
public class Auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        HardwareSoftware hw = new HardwareSoftware();
        hw.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0,0,0));


        TrajectorySequence path1 = drive.trajectorySequenceBuilder(new Pose2d(0,0,0))
                .forward(26)
                .turn(Math.toRadians(90))
                .forward(40)
                .build();

        TrajectorySequence path2 = drive.trajectorySequenceBuilder(path1.end())
                .back(4)
                .strafeRight(24)
                .build();


        waitForStart();

        drive.followTrajectorySequence(path1);

        sleep(500);

        sleep(1000);

        drive.followTrajectorySequence(path2);


    }
}


