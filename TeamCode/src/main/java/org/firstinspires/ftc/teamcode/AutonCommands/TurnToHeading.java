package org.firstinspires.ftc.teamcode.AutonCommands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Command;
import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class TurnToHeading extends Command {
    private ElapsedTime timer;
    private double startTime;
    private double timeOut;
    private HackinHoundsHardware robot;
    private double powerLevel;
    private double desiredAngle;
    private double deltaAngle;
    double turnSign;

    public TurnToHeading(Hardware robot, ElapsedTime timer, double heading, double powerLevel, double timeOut) {
        super(robot);
        this.robot = (HackinHoundsHardware) getRobot();
        //testRobot.resetAngle();
        this.timer = timer;
        if (timeOut < 0) {
            timeOut = 30.0;
        }
        this.timeOut = timeOut*1000;
        this.powerLevel = powerLevel;
        this.desiredAngle = heading;
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
            this.turnSign = Math.signum(desiredAngle - robot.getAngle());
            setState(RUNNING);
        }
    }

    public void run() {
        if (getState() == RUNNING) {
            double elapsedTime = timer.milliseconds()-startTime;
            double currentAngle = robot.getAngle();
            double angleError = desiredAngle - currentAngle;
            // Message: It originally was if > 0, Now trying to do > 0.5 for to speed it up
            if (turnSign * angleError > 0.5 && elapsedTime < timeOut) {
                double powerFactor = Math.min(Math.abs((turnSign*angleError)/45.0),1.0);
//                if (turnSign >= 0) {
                    robot.leftFront.setPower(-turnSign*powerLevel*powerFactor);
                    robot.rightFront.setPower(turnSign*powerLevel*powerFactor);
                    robot.leftBack.setPower(-turnSign*powerLevel*powerFactor);
                    robot.rightBack.setPower(turnSign*powerLevel*powerFactor);
//                } else {
//                    robot.leftFront.setPower(-turnSign*powerLevel*powerFactor);
//                    robot.rightFront.setPower(turnSign*powerLevel*powerFactor);
//                    robot.leftBack.setPower(-turnSign*powerLevel*powerFactor);
//                    robot.rightBack.setPower(turnSign*powerLevel*powerFactor);
//                }
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
