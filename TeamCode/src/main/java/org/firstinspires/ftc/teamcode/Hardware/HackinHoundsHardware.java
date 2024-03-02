package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

// Generic robot class
public class HackinHoundsHardware extends Hardware {
    public HardwareMap robotMap;

    // Drivetrain Members
    public DcMotorEx  leftFront;
    public DcMotorEx  rightFront;
    public DcMotorEx  leftBack;
    public DcMotorEx  rightBack;
    public DcMotorEx slide;
    public Servo wrist;
    public Servo top_claw;
    public Servo bottom_claw;
    public Servo launcher;
    public HuskyLens huskyLens;
    public ModernRoboticsI2cRangeSensor rangeSensor;
    public ColorSensor colorSensor;
    public Rev2mDistanceSensor distance;
    public RevBlinkinLedDriver Lights;
    public Servo hook;
    public DcMotorEx spool;
    private double lastAngle;

    public IMU imu;
    public YawPitchRollAngles angles;

    private YawPitchRollAngles lastAngles;
    private double globalAngle;

    // 1000 ticks was roughly 18 in.
    public static final double TICK_PER_INCH = 500.0/12.0;
    public static final double MinPower = 0.35;

    public enum ScoringPosition {LEFT, CENTER, RIGHT, NONE}
    public int[] TagPosition = {4, 6, 2};

    OpenCvCamera camera;

    static final double FEET_PER_METER = 3.28084;

    // UNITS ARE METERS
    double tagsize = 0.166;


    /* Constructor */
    public HackinHoundsHardware(){

    }

    // Override to set actual robot configuration
    public void init(HardwareMap hwMap) {

        // Save reference to Hardware map
        robotMap = hwMap;

        // Define and Initialize Motors for drivetrain
        leftFront  = robotMap.get(DcMotorEx.class, "left_front");
        rightFront = robotMap.get(DcMotorEx.class, "right_front");
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack = robotMap.get(DcMotorEx.class, "left_back");
        rightBack = robotMap.get(DcMotorEx.class, "right_back");
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide = robotMap.get(DcMotorEx.class, "slide");
//        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //slide.setDirection(DcMotorSimple.Direction.REVERSE);

        spool = robotMap.get(DcMotorEx.class, "spool");
        spool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wrist = robotMap.get(Servo.class, "wrist");
        bottom_claw = robotMap.get(Servo.class, "top_claw");
        top_claw = robotMap.get(Servo.class, "bottom_claw");
        //launcher = robotMap.get(Servo.class, "launcher");
        //hook = robotMap.get(Servo.class, "hook");


        //IMportant plugged in areas thingies
        //Expansion Hub:
        //0 - hook
        //2 - launcher

        //Control Hub:
        //0 - lights

        huskyLens = robotMap.get(HuskyLens.class, "huskylens");

        rangeSensor = robotMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");

        colorSensor = robotMap.get(ColorSensor.class, "color");
        distance = robotMap.get(Rev2mDistanceSensor.class, "distance");

        //Lights = robotMap.get(RevBlinkinLedDriver.class, "lights");

        // Defines the REV Hub's internal IMU (Gyro)
        imu = robotMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        lastAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public double clamp(double x, double min, double max) {
        return Math.max(min,Math.min(max,x));
    }

    public double getAngle() {
        double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        double deltaAngle = angle - lastAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngle = angle;

        return globalAngle;
    }
}

