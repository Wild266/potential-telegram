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
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name="JavaGyroTest", group="Linear Opmode")
public class Team8535GyroTest extends LinearOpMode {

    private boolean prodbot = false;
    public static boolean JOYSTICK_SCALING=true;

    //Speed Factor for Fast/Slow Mode
    private double speedFactor = 1.0; //default full speed

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private double lastLoopTime=0.0;
    private double currentLoopTime=0.0;
    private double lastHold=0.0;

    //Drive Motors
    private DcMotor lf=null;
    private DcMotor rf=null;
    private DcMotor lb=null;
    private DcMotor rb=null;

    int targetAngle=0;

    //Gyro Sensor
    ModernRoboticsI2cGyro gyro; //a gyro would be really useful

    private DcMotor getMotor(String motorName) { //these could be made generic using type notation
        try {
            return(hardwareMap.get(DcMotor.class,motorName));
        } catch (Exception e) {
            return(null);
        }
    }

    private I2cDevice getDevice(String deviceName) { //these could be made generic using type notation
        try {
            return(hardwareMap.get(I2cDevice.class,deviceName));
        } catch (Exception e) {
            return(null);
        }
    }

    private ModernRoboticsI2cGyro getGyro(String gyroName) { //these could be made generic using type notation
        try {
            return(hardwareMap.get(ModernRoboticsI2cGyro.class,gyroName));
        } catch (Exception e) {
            return(null);
        }
    }

    private void mecanumMove(double lsx,double lsy,double rsx) {
        if (JOYSTICK_SCALING) {
            lsy=Math.pow(lsy,5.0);
            lsx=Math.pow(lsx,5.0);
            rsx=Math.pow(rsx,5.0);
        }
        mecanumMoveNoScale(lsx,lsy,rsx);
    }

    /**
     * Set motors for a mecanum move
     * @param lsx left stick x (left/right)
     * @param lsy left stick y (front/back)
     * @param rsx right stick x (rotation)
     */
    private void mecanumMoveNoScale(double lsx,double lsy,double rsx) {

        double r = Math.sqrt(lsy*lsy+lsx*lsx);
        double robotAngle = Math.atan2(-1*lsy,lsx) - Math.PI / 4;
        double rightX = rsx;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        lf.setPower(v1);
        rf.setPower(v2);
        lb.setPower(v3);
        rb.setPower(v4);
    }

    private void gyroTurn(int degrees,ElapsedTime runtime, double maxTime) {
        int current=gyro.getHeading();
        int target=current+degrees;
        telemetry.addData("Gyro Turn","At %d Target %d",current,target);
        telemetry.update();
        double startTime=runtime.time();
        if (degrees>0) {
            mecanumMoveNoScale(0,0,-0.25);
            while(gyro.getHeading()<target) {
                telemetry.addData("Turning","At %d",gyro.getHeading());
                telemetry.update();
                if ((runtime.time()-startTime)>maxTime) break;
            }
        } else {
            mecanumMoveNoScale(0,0,0.25);
            while(gyro.getHeading()>target) {
                telemetry.addData("Turning","At %d",gyro.getHeading());
                telemetry.update();
                if ((runtime.time()-startTime)>maxTime) break;
            }
        }
        mecanumMoveNoScale(0,0,0);
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        if (getDevice("drivebot") != null) {
            prodbot = false;
            telemetry.addData("Bot", "DriveBot");
        } else {
            prodbot = true;
            telemetry.addData("Bot", "ProdBot");
        }

        lf  = hardwareMap.get(DcMotor.class, "lf");
        rf  = hardwareMap.get(DcMotor.class, "rf");
        lb  = hardwareMap.get(DcMotor.class, "lb");
        rb  = hardwareMap.get(DcMotor.class, "rb");

        gyro = getGyro("gyro");
        if (gyro!=null) { //just rename the gyro in the resource file to run without it
            //gyro.setI2cAddress(I2cAddr.create7bit(0x10)); //we believe these are 7bit addresses
            telemetry.log().add("Gyro Calibrating. Do Not Move!");
            gyro.calibrate();
            // Wait until the gyro calibration is complete
            runtime.reset();
            while (!isStopRequested() && gyro.isCalibrating())  {
                telemetry.addData("Calibrating", "%s", Math.round(runtime.seconds())%2==0 ? "|.." : "..|");
                telemetry.update();
                sleep(50);
            }
            telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
            telemetry.clear(); telemetry.update();
            gyro.resetZAxisIntegrator();
        }

        lf.setDirection(DcMotor.Direction.REVERSE); //was REVERSE
        rf.setDirection(DcMotor.Direction.REVERSE); //was REVERSE
        lb.setDirection(DcMotor.Direction.FORWARD); //was FORWARD
        rb.setDirection(DcMotor.Direction.FORWARD); //was FORWARD

        if (prodbot) {
            lf.setDirection(DcMotor.Direction.REVERSE); //was REVERSE
            rf.setDirection(DcMotor.Direction.REVERSE); //was REVERSE
            lb.setDirection(DcMotor.Direction.REVERSE); //was FORWARD
            rb.setDirection(DcMotor.Direction.FORWARD); //was FORWARD
        } else {
            lf.setDirection(DcMotor.Direction.REVERSE); //was REVERSE
            rf.setDirection(DcMotor.Direction.FORWARD); //was REVERSE
            lb.setDirection(DcMotor.Direction.REVERSE); //was FORWARD
            rb.setDirection(DcMotor.Direction.FORWARD); //was FORWARD
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        double time = 0; //move timer

        while (opModeIsActive()) {
            if (gamepad1.right_stick_x<-0.3) {
                targetAngle--;
                if (targetAngle<0) targetAngle=0;
            } else if (gamepad1.right_stick_x>0.3) {
                targetAngle++;
                if (targetAngle>359) targetAngle=359;
            }
            telemetry.addData("Target Angle",targetAngle);
            telemetry.update();
            sleep(500);
        }
    }
}
