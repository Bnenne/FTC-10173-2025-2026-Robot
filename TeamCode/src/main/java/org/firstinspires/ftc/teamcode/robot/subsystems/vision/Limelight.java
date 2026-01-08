package org.firstinspires.ftc.teamcode.robot.subsystems.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.subsystems.Subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class Limelight implements Subsystem {
    Limelight3A limelight;
    CameraState currentState;
    int desiredPipeline;

    LLResult result;
    public Results results;

    public enum CameraState {
        IDLE
    }

    public static class Results {
        public boolean hasTarget;
        public double tx;
        public double ty;
        public double ta;
    }


    // Constructor
    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

        // Default to pipeline 0
        desiredPipeline = 0;
        limelight.pipelineSwitch(0);

        currentState = CameraState.IDLE;
    }

    public Limelight setPipeline(int pipelineIndex) {
        desiredPipeline = pipelineIndex;
        limelight.pipelineSwitch(pipelineIndex);
        return this;
    }

    public void periodic() {
        result = limelight.getLatestResult();
        if (result.getPipelineIndex() == desiredPipeline) {
            if (result != null && result.isValid()) {
                results.hasTarget = true;
                results.tx = result.getTx(); // How far left or right the target is (degrees)
                results.ty = result.getTy(); // How far up or down the target is (degrees)
                results.ta = result.getTa(); // How big the target looks (0%-100% of the image)
            } else {
                results.hasTarget = false;
            }
        }
    }

    public void updateTelemetry(Telemetry telemetry) {
        if (results.hasTarget) {
            telemetry.addData(getName() + " Target X", results.tx);
            telemetry.addData(getName() + " Target Y", results.ty);
            telemetry.addData(getName() + " Target Area", results.ta);
        } else {
            telemetry.addData(getName(), "No Targets");
        }
    }

    public boolean isHealthy() {
        return limelight != null && result.isValid();
    }

    public void stop() {
        Subsystem.super.stop();
    }
}
