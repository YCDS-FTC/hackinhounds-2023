package org.firstinspires.ftc.teamcode.AutonCommands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Command;
import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

public class RepositionAndSlide extends Command {
    private ElapsedTime timer;
    private double startTime;
    private double timeOut;
    private HackinHoundsHardware robot;
    private double powerLevel;
    private double sPowerLevel;
    private int position;
    private boolean slideDone = false;

    private int leftFrontTicks;
    private int rightFrontTicks;
    private int leftFrontStartTicks;
    private int rightFrontStartTicks;
    private int leftEndTicks;
    private int rightEndTicks;
    private double distance;
    private double measure;

    public RepositionAndSlide(Hardware robot, ElapsedTime timer, double distance, double timeOut, double powerLevel, int position, double sPowerLevel) {
        super(robot);
        this.timer = timer;
        this.distance = distance;
        if (timeOut<0){
            timeOut=30;
        }
        this.timeOut = timeOut*1000;
        this.powerLevel = powerLevel;
        this.robot = (HackinHoundsHardware) getRobot();
        this.sPowerLevel = sPowerLevel;
        this.position = position;
        leftFrontTicks=0;
        rightFrontTicks=0;
        setState(STARTING);
    }

    public void init() {
        if (getState() == STARTING) {
            setState(RUNNING);
            startTime = timer.milliseconds();
            robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftFrontStartTicks=robot.leftFront.getCurrentPosition();
            rightFrontStartTicks=robot.rightFront.getCurrentPosition();
//            leftEndTicks=leftFrontTicks + (int)(-(inches*HackinHoundsHardware.TICK_PER_INCH)*Math.signum(powerLevel));
//            rightEndTicks=rightFrontTicks + (int)(-(inches*HackinHoundsHardware.TICK_PER_INCH)*Math.signum(powerLevel));
        }
    }


    public void run() {
        if (getState() == RUNNING) {
            double elapsedTime = timer.milliseconds()-startTime;
            if (elapsedTime > timeOut) {
                setState(ENDING);
            } else {
                measure = robot.rangeSensor.getDistance(DistanceUnit.INCH);
                if (Math.abs(robot.slide.getCurrentPosition()-position) > 10) {
                    robot.slide.setTargetPosition(position);
                    robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.slide.setPower(sPowerLevel);
                } else {
                    robot.slide.setPower(0);
                    slideDone = true;
                }
                if (measure > distance + 2) {
                    robot.leftFront.setPower(powerLevel);
                    robot.leftBack.setPower(-powerLevel);
                    robot.rightFront.setPower(-powerLevel);
                    robot.rightBack.setPower(powerLevel);
                } else if (measure < distance - 2) {
                    robot.leftFront.setPower(-powerLevel);
                    robot.leftBack.setPower(powerLevel);
                    robot.rightFront.setPower(powerLevel);
                    robot.rightBack.setPower(-powerLevel);
                } else {
                    robot.leftFront.setPower(0);
                    robot.leftBack.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.rightBack.setPower(0);
                    if (slideDone) {
                        setState(ENDING);
                    }
                }
            }
        }
    }

    public void end() {
        if (getState() == ENDING) {
        }
        setState(DONE);
    }

}
