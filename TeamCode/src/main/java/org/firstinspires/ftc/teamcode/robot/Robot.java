package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.subsystems.Drive;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.LED;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.VisionController;

public class Robot {
    public final Drive drive;
    public final Shooter shooter;
    public final Intake intake;
    public final LED led;
    public final VisionController vision;

    public Robot(HardwareMap hardwareMap) {
        led = new LED(hardwareMap);
        shooter = new Shooter(hardwareMap, led);
        intake = new Intake(hardwareMap, led);
        drive = new Drive(hardwareMap);
        vision = new VisionController(hardwareMap);
    }
}
