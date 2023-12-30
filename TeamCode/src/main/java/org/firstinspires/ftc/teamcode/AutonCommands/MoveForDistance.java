package org.firstinspires.ftc.teamcode.AutonCommands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Command;
import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;

import java.util.ArrayList;
import java.util.Collections;

public class MoveForDistance extends Command {
    private ElapsedTime timer;
    private double startTime;
    private double timeOut;
    private double riseTicks;
    private double fallTicks;
    private double totalTicks;
    private HackinHoundsHardware robot;
    private double powerLevel;
    private double powerDirection;
    private double diff;
    private boolean useInit;
    private boolean useEnd;

    private int FleftTicks;
    private int FrightTicks;
    private int FleftStartTicks;
    private int FrightStartTicks;
    private int FleftEndTicks;
    private int FrightEndTicks;

    private int BleftTicks;
    private int BrightTicks;
    private int BleftStartTicks;
    private int BrightStartTicks;
    private int BleftEndTicks;
    private int BrightEndTicks;

    private double inches;

    private ArrayList<Integer> currentTicks;

    public MoveForDistance(Hardware robot, double totalDistance, double riseDistance, double fallDistance, ElapsedTime timer, double timeOut, double powerLevel, double diff) {
        super(robot);
        this.timer = timer;
        this.inches=totalDistance;
        if (timeOut < 0) {
            timeOut = 30;
        }
        this.timeOut = timeOut * 1000;
        this.totalTicks = totalDistance * HackinHoundsHardware.TICK_PER_INCH;
        this.riseTicks = riseDistance * HackinHoundsHardware.TICK_PER_INCH;
        this.fallTicks = fallDistance * HackinHoundsHardware.TICK_PER_INCH;
        if (powerLevel < 0) {
            powerDirection = -1.0;
        } else {
            powerDirection = 1.0;
        }
        this.powerLevel = Math.abs(powerLevel);
        this.diff = diff;
        this.robot = (HackinHoundsHardware) getRobot();
        this.useInit = true;
        this.useEnd = true;

        // Set all encoder ticks to 0
        FleftTicks = 0;
        FrightTicks = 0;
        BleftTicks = 0;
        BrightTicks = 0;

        currentTicks = new ArrayList<Integer>();
        currentTicks.add(0);
        currentTicks.add(0);
        currentTicks.add(0);
        currentTicks.add(0);

        setState(STARTING);
    }

    public MoveForDistance(Hardware robot, double totalDistance, double riseDistance, double fallDistance, ElapsedTime timer, double timeOut, double powerLevel, double diff, boolean useInit, boolean useEnd) {
        super(robot);
        if (timeOut < 0) {
            timeOut = 30;
        }
        this.timeOut = timeOut * 1000;
        this.timer = timer;
        this.totalTicks = totalDistance * HackinHoundsHardware.TICK_PER_INCH;
        this.riseTicks = riseDistance * HackinHoundsHardware.TICK_PER_INCH;
        this.fallTicks = fallDistance * HackinHoundsHardware.TICK_PER_INCH;
        this.powerLevel = powerLevel;
        this.diff = diff;
        this.robot = (HackinHoundsHardware) getRobot();
        this.useInit = useInit;
        this.useEnd = useEnd;

        // Set all encoder ticks to 0
        FleftTicks = 0;
        FrightTicks = 0;
        BleftTicks = 0;
        BrightTicks = 0;

        currentTicks = new ArrayList<Integer>();
        currentTicks.add(0);
        currentTicks.add(0);
        currentTicks.add(0);
        currentTicks.add(0);

        setState(STARTING);
    }

    public void init() {
        if (getState() == STARTING && useInit) {
            startTime = timer.milliseconds();

            // Reset drive motors
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Set the left and right starting ticks to their current position
            FleftStartTicks = robot.leftFront.getCurrentPosition();
            FrightStartTicks = robot.rightFront.getCurrentPosition();
            FleftEndTicks=FleftStartTicks + (int)((inches*HackinHoundsHardware.TICK_PER_INCH)*Math.signum(powerLevel));
            FrightEndTicks=FrightStartTicks + (int)((inches*HackinHoundsHardware.TICK_PER_INCH)*Math.signum(powerLevel));

            BleftStartTicks = robot.leftFront.getCurrentPosition();
            BrightStartTicks = robot.rightFront.getCurrentPosition();
            BleftEndTicks=BleftStartTicks + (int)((inches*HackinHoundsHardware.TICK_PER_INCH)*Math.signum(powerLevel));
            BrightEndTicks=BrightStartTicks + (int)((inches*HackinHoundsHardware.TICK_PER_INCH)*Math.signum(powerLevel));
        }
        setState(RUNNING);
    }

    public void run() {
        if (getState() == RUNNING) {
            double elapsedTime = timer.milliseconds() - startTime;
            double power = HackinHoundsHardware.MinPower;
            if (elapsedTime > timeOut) {
                // When it is finished, set state to ending
                setState(ENDING);
            } else {
                int currentFLeftTicks = Math.abs(robot.leftFront.getCurrentPosition() - FleftStartTicks);
                currentTicks.set(0, currentFLeftTicks);
                int currentFRightTicks = Math.abs(robot.rightFront.getCurrentPosition() - FrightStartTicks);
                currentTicks.set(1, currentFRightTicks);
                int currentBLeftTicks = Math.abs(robot.leftFront.getCurrentPosition() - BleftStartTicks);
                currentTicks.set(2, currentBLeftTicks);
                int currentBRightTicks = Math.abs(robot.rightFront.getCurrentPosition() - BrightStartTicks);
                currentTicks.set(3, currentBRightTicks);

                // This is the math behind the rise and fall distance which makes our trapezoidal movement when the power level is positive going forward, you can see more about it in our engineering portfolio/notebook
                Integer d = 0;
                for (Integer i : currentTicks) {
                    d += i;
                }
                d = d/currentTicks.size();

                if (d < riseTicks) {
                    // Rising power
                    power = ((powerLevel - HackinHoundsHardware.MinPower)/riseTicks)*d + HackinHoundsHardware.MinPower;
                } else if (d < totalTicks-fallTicks) {
                    // Constant power
                    power = powerLevel;
                } else {
                    // Falling power
                    power = ((HackinHoundsHardware.MinPower-powerLevel)/(fallTicks))*(d - (totalTicks-fallTicks)) + powerLevel;
                }
                if (Math.signum(powerLevel)*(FleftEndTicks - currentFLeftTicks) < 0  || Math.signum(powerLevel)*(FrightEndTicks - currentFRightTicks) < 0 || Math.signum(powerLevel)*(BrightEndTicks - currentBRightTicks) < 0 || Math.signum(powerLevel)*(BleftEndTicks - currentBLeftTicks) < 0){
                    setState(ENDING);
                } else {
                    robot.leftFront.setPower(power * powerDirection);
                    robot.leftBack.setPower(power * powerDirection);
                    robot.rightFront.setPower(power * powerDirection);
                    robot.rightBack.setPower(power * powerDirection);
                }
            }
        }
    }

    public void end() {
        if (getState() == ENDING && useEnd) {
            // Set power to 0 when in the ending mode
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);
        }
        setState(DONE);
    }

}
