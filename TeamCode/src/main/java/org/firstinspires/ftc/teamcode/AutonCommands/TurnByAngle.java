package org.firstinspires.ftc.teamcode.AutonCommands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Command;
import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class TurnByAngle extends Command {
    private ElapsedTime timer;
    private double startTime;
    private double timeOut;
    private HackinHoundsHardware robot;
    private double powerLevel;
    private double desiredAngle;
    private double deltaAngle;
    double turnSign;

    public TurnByAngle(Hardware robot, ElapsedTime timer, double deltaAngle, double powerLevel, double timeOut) {
        super(robot);
        this.robot = (HackinHoundsHardware) getRobot();

        // turnSign is positive for right turn
        turnSign = Math.signum(deltaAngle);
        //testRobot.resetAngle();
        this.timer = timer;
        if (timeOut < 0) {
            timeOut = 30.0;
        }
        this.timeOut = timeOut*1000;
        this.powerLevel = powerLevel;
        this.deltaAngle = deltaAngle;
        setState(STARTING);
    }

    public void init() {
        if (getState() == STARTING) {
            startTime = timer.milliseconds();
            robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Right turn (positive delta) is negative direction for gyro
            //this.desiredAngle = robot.getAngle()-deltaAngle;  //  BY ANGLE
            this.desiredAngle = -deltaAngle;    // TO ANGLE

            setState(RUNNING);
        }
    }

    public void run() {
        if (getState() == RUNNING) {
            double elapsedTime = timer.milliseconds()-startTime;
            double currentAngle = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            // Gyro angle gets more negative turning right, but user angles are positive turning right
            double angleError = desiredAngle-currentAngle;
            if (Math.abs(angleError) > 0.5 && elapsedTime < timeOut) {
                //turnSign = Math.signum(angleError);
                currentAngle = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                angleError = desiredAngle-currentAngle;
                double powerFactor = Math.min((-turnSign*angleError)/45.0+0.1,1.0);
                // turnSign = 1 is right turn
                robot.leftFront.setPower(turnSign*powerLevel*powerFactor);
                robot.rightFront.setPower(-turnSign*powerLevel*powerFactor);
                robot.leftBack.setPower(turnSign*powerLevel*powerFactor);
                robot.rightBack.setPower(-turnSign*powerLevel*powerFactor);
            } else {
                setState(ENDING);
            }
        }
    }

    public void end() {
        if (getState() == ENDING) {
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);
        }
        setState(DONE);
    }

}
