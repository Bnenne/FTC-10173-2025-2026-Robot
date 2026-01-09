package org.firstinspires.ftc.teamcode.robot.autos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotState;

public class AutoManager {
    MecanumDrive drive;
    RobotState robotState;
    Robot robot;
    public Pose2d currentPose;
    int patternID;
    public enum Alliance {
        RED,
        BLUE
    }
    Alliance alliance;

    public enum IntakePose {
        GPP(21, 36),
        PGP(22, 12),
        PPG(23, -12);

        private final int id;
        private final int x;

        IntakePose(int id, int x) {
            this.id = id;
            this.x = x;
        }

        public int getX() {
            return x;
        }

        public int getId() {
            return id;
        }

        public static IntakePose fromId(int id) {
            for (IntakePose p : values()) {
                if (p.id == id) return p;
            }
            throw new IllegalArgumentException("Invalid ShotPose id: " + id);
        }
    }


    public AutoManager(Pose2d beginPose, HardwareMap hardwareMap, Alliance alliance) {
        drive = new MecanumDrive(hardwareMap, beginPose);
        robotState = new RobotState();
        robot = new Robot(hardwareMap, robotState);
        currentPose = beginPose;
        this.alliance = alliance;
    }

    public void setPose(Pose2d pose) {
        currentPose = pose;
    }

    public void getMotif() {
    }

    public SequentialAction shoot(double power, int feedDuration) {
        return new SequentialAction(
                robot.shooter.spinUp(power, 1.0),
                robot.intake.feed(1, feedDuration),
                robot.shooter.spinUp(0.0, -1)
        );
    }

    public ParallelAction moveAndShoot(double power, int feedDuration, Pose2d newPose) {
        return new ParallelAction(
                // move to new pose
                drive.actionBuilder(new Pose2d(currentPose.position.x, currentPose.position.y, currentPose.heading.real))
                        .strafeToLinearHeading(new Vector2d(newPose.position.x, newPose.position.y), newPose.heading.real)
                        .build(),
                // spin up shooter while moving
                robot.shooter.spinUp(power, 1.0),

                // feed artifacts after reaching new pose and spinning up
                new SequentialAction(
                        robot.intake.feed(1, feedDuration),
                        robot.shooter.spinUp(0.0, -1)
                )
        );
    }

    public SequentialAction straightIntake() {

        double strafeY = (alliance == Alliance.BLUE) ? -55 : 55;

        return new SequentialAction(
                robot.intake.intake(1),
                drive.actionBuilder(new Pose2d(currentPose.position.x, currentPose.position.y, currentPose.heading.real))
                        .strafeToLinearHeading(new Vector2d(currentPose.position.x, strafeY), currentPose.heading.real, new TranslationalVelConstraint(17.5))
                        .build(),
                robot.intake.intake(0)
        );
    }

    public Action updatePose(Pose2d pose) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // update the current pose
                currentPose = pose;

                // returns false to indicate action is complete
                return false;
            }
        };
    }
}
