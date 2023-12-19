package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;

// Generic robot class
public class VisionTestHardware extends Hardware {
    public HardwareMap robotMap;

    public static final int READ_PERIOD = 1;

    public HuskyLens huskyLens;

    public void init(HardwareMap hwMap) {
        robotMap = hwMap;
        huskyLens = robotMap.get(HuskyLens.class, "huskylens");
    }

}

