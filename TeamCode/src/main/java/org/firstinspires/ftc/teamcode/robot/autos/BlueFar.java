package org.firstinspires.ftc.teamcode.robot.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Constants;

@Autonomous(name="Blue Far", group="2025-2026")
public final class BlueFar extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // starting pose
        Pose2d beginPose = new Pose2d(64, -16, Math.toRadians(180));

        Pose2d BLUE_CLOSE = Constants.ShootingPoses.BLUE_CLOSE;
        Pose2d BLUE_FAR = Constants.ShootingPoses.BLUE_FAR;

        double CLOSE_SHOOTER_POWER = Constants.ShootingPower.CLOSE;
        double FAR_SHOOTER_POWER = Constants.ShootingPower.FAR;

        AutoBuilder autoBuilder = new AutoBuilder(hardwareMap, beginPose, AutoBuilder.Alliance.BLUE);

        waitForStart();

        autoBuilder
                .moveAndShoot(FAR_SHOOTER_POWER, 5000, BLUE_FAR)
                .alignWithArtifacts(21)
                .straightIntake()
                .moveAndShoot(FAR_SHOOTER_POWER, 5000, BLUE_FAR)
                .build();
    }
}
