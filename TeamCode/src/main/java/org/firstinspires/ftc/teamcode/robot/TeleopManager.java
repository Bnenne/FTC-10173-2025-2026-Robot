package org.firstinspires.ftc.teamcode.robot;

public class TeleopManager {
    private final Robot robot;
    private final DriverControls controls;

    public TeleopManager(Robot robot, DriverControls controls) {
        this.robot = robot;
        this.controls = controls;
    }

    public void update() {
        robot.vision.setManualExposure();

        if (controls.resetYawPressed()) {
            robot.drive.resetYaw();
        }

        if (controls.lockDrivePressed()) {
            robot.drive.lock(robot.vision.getBearing(0));
            robot.drive.setDrivePowers(controls.driver, true);
        } else {
            robot.drive.setDrivePowers(controls.driver, false);
        }

        robot.intake.setPower(controls.intakePower(),
                controls.fullIntakePressed(),
                controls.halfOuttakePressed());

        robot.shooter.setPower(robot.vision.getDistance(0));
        robot.shooter.spin(controls.spinShooterPressed());
        robot.led.update();
    }
}
