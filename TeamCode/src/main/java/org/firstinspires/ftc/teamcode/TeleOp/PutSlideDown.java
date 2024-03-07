/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.HackinHoundsHardware;



/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="PutSlideDown", group="Linear OpMode")

public class PutSlideDown extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private HackinHoundsHardware robot = new HackinHoundsHardware();
    double shift = 1;
    boolean slideMoving = false;
    boolean wristMoving = false;
    boolean hold = true;
    int currentPos = 0;

    //PID Stuff
    double integralSum = 0;
    double Kp = 25;    // default 10
    double Ki = 3;     // default 3
    double Kd = 0.1;   // default 0
    double Kf = 0;     // default 0
    double lasterror = 0;

    PIDFCoefficients pidOld;
    PIDFCoefficients pidNew;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        pidOld = robot.slide.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("P,I,D,F", "%.04f, %.04f, %.04f, %.04f", pidOld.p, pidOld.i, pidOld.d, pidOld.f);
        telemetry.update();
        pidNew = new PIDFCoefficients(Kp, Ki, Kd, Kf);
        robot.slide.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        waitForStart();
        runtime.reset();
        robot.colorSensor.enableLed(false);
        while (opModeIsActive()) {
            double slidePower = gamepad2.left_stick_y;
            int slideCurrentPos = robot.slide.getCurrentPosition();
            if ((slidePower < -0.1) && (slideCurrentPos > -11000)) {
                robot.slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.slide.setPower(slidePower);
            } else if ((slidePower > 0.1) && (slideCurrentPos < 11000)) {
                robot.slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.slide.setPower(slidePower);
            } else {
                robot.slide.setPower(0);
            }

            if (gamepad2.a) {
                robot.spool.setPower(-1);
            } else {
                robot.spool.setPower(0);
            }

            if (gamepad2.y) {
                robot.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            double wristPower = gamepad2.right_stick_y * 0.05;
            robot.wrist.setPosition(clamp(robot.wrist.getPosition() + wristPower,0.325, 0.36));

            //Telemetry
            telemetry.addData("slide Pos:", "%d", robot.slide.getCurrentPosition());
            telemetry.update();
        }
    }
    private double clamp(double x, double min, double max) {

        return Math.max(min,Math.min(max,x));
    }
}
