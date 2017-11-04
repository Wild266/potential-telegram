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

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * Team8235 TeleOp OpMode
 */

@TeleOp(name="JavaTeleOp", group="Linear Opmode")
public class Team8535JavaTeleOp extends LinearOpMode {

    private boolean prodbot = false;

    //Vision
    VuforiaLocalizer vuforia;

    //Speed Factor for Fast/Slow Mode
    private double speedFactor = 1.0; //default full speed

    private ElapsedTime runtime = new ElapsedTime();

    //Drive Motors
    private DcMotor lf=null;
    private DcMotor rf=null;
    private DcMotor lb=null;
    private DcMotor rb=null;

    //Front Gripper
    private DcMotor gripperLiftMotor=null;
    private Servo gripperLeftServo=null;
    private Servo gripperRightServo=null;

    //Relic Arm
    private DcMotor vacuumMotor=null;
    private DcMotor armExtendMotor=null;
    private DcMotor armLiftMotor=null;
    private Servo vacuumReleaseServo=null;

    //Ball Arm
    private Servo ballArmServo=null;
    private ColorSensor ballColorSensor=null;

    private static boolean SHOW_CAMERA=true; //whether to show the camera on the phone screen
    private static boolean JOYSTICK_SCALING=true; //whether to scale joystick values by cubing value (more precision for small movements)

    //introduce methods which allow us to continue even when particular devices aren't found (by just not utilizing those devices)

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

    private Servo getServo(String servoName) { //these could be made generic using type notation
        try {
            return(hardwareMap.get(Servo.class,servoName));
        } catch (Exception e) {
            return(null);
        }
    }

    private ColorSensor getColorSensor(String sensorName) { //these could be made generic using type notation
        try {
            return(hardwareMap.get(ColorSensor.class,sensorName));
        } catch (Exception e) {
            return(null);
        }
    }

    @Override
    public void runOpMode() {

        VuforiaLocalizer.Parameters parameters=null;
        if (SHOW_CAMERA) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        } else {
            parameters = new VuforiaLocalizer.Parameters();
        }

        parameters.vuforiaLicenseKey = "Ab01nl7/////AAAAGeQnfaGoXUZ+i+4cRvO5jFNG9p0WO71bT/iVJiyCR32g6mazT1g6HiB2OmYcVTUVAWWGDIMKhNlGGjHAS/MCdmgK9VR4jbeUxBD0HT1xXebg7sD5+o2+4HSKheLgOnGdjVMwuUZK/3pnthEADVlvUZsDtrIxxYKBQEQSTf3uWP6vYFTax3kjPSIczUrmjUh6HhIhEm8NcrP4FgE/IjOr4xABtOU8QK4pdMDSxI5UatrszXVfs5jeUJ1gsciJBhwb95YN3e5Eqp/Mhr0K4iqdfGlPZLSYsm2757vfocnlHXaCM1jaU6jM42f8PR0/FLqZX9nIDSbtj+LAo9ufa6qi5/gnW3Ps3Vm1xpiGr7Tp10WN";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; //use the back camera for VuForia
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        lf  = hardwareMap.get(DcMotor.class, "lf");
        rf  = hardwareMap.get(DcMotor.class, "rf");
        lb  = hardwareMap.get(DcMotor.class, "lb");
        rb  = hardwareMap.get(DcMotor.class, "rb");

        gripperLiftMotor=getMotor("gripper_lift");
        gripperLeftServo=getServo("gripper_left");
        gripperRightServo=getServo("gripper_right");
        vacuumMotor=getMotor("vacuum");
        armExtendMotor=getMotor("arm_extend");
        armLiftMotor=getMotor("arm_lift");
        vacuumReleaseServo=getServo("vacuum_release");
        ballArmServo=getServo("ball_arm");
        ballColorSensor=getColorSensor("ball_color");

        if (getDevice("drivebot") != null) {
            prodbot = false;
        } else {
            prodbot = true;
        }
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        if (prodbot) {
            lf.setDirection(DcMotor.Direction.FORWARD); //was REVERSE
            rf.setDirection(DcMotor.Direction.REVERSE); //was REVERSE
            lb.setDirection(DcMotor.Direction.REVERSE); //was FORWARD
            rb.setDirection(DcMotor.Direction.REVERSE); //was FORWARD
        } else {
            lf.setDirection(DcMotor.Direction.REVERSE); //was REVERSE
            rf.setDirection(DcMotor.Direction.REVERSE); //was REVERSE
            lb.setDirection(DcMotor.Direction.FORWARD); //was FORWARD
            rb.setDirection(DcMotor.Direction.FORWARD); //was FORWARD
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoders
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //runs again
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        relicTrackables.activate();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //translate some gamepad controls to variables

            //gamepad1
            double backForward=gamepad1.left_stick_y;
            double leftRight=gamepad1.left_stick_x;
            double ccwCwRotate=gamepad1.right_stick_x;
            boolean slowMode=(gamepad1.left_trigger>0.0);
            boolean lowerBallArm=(gamepad1.dpad_down);
            boolean raiseBallArm=(gamepad1.dpad_up);

            //gamepad2
            double lowerRaiseLift=gamepad2.left_stick_y;
            double leftClamp=gamepad2.left_trigger;
            double rightClamp=gamepad2.right_trigger;
            boolean height1=gamepad2.x; //lowest preset gripper height
            boolean height2=gamepad2.a;
            boolean height3=gamepad2.b;
            boolean height4=gamepad2.y; //highest preset gripper height
            boolean extendRelicArm=gamepad2.dpad_right;
            boolean retractRelicArm=gamepad2.dpad_left;
            boolean raiseRelicArm=gamepad2.dpad_up;
            boolean lowerRelicArm=gamepad2.dpad_down;
            double lowerRaiseArm=gamepad2.right_stick_y;
            boolean startVacuum=gamepad2.start;
            boolean stopVacuum=gamepad2.back;

            if (gamepad1.y) { //stop and look for vumarks if Y key is down

                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                    telemetry.addData("VuMark", "%s visible", vuMark);
                }


            } else {

                if (gamepad1.left_trigger > 0.0) {
                    speedFactor = 0.75;
                } else {
                    speedFactor = 1.0;
                }

                double lsy=gamepad1.left_stick_y*speedFactor;
                double lsx=gamepad1.left_stick_x*speedFactor;
                double rsx=gamepad1.right_stick_x*(speedFactor*0.85);

                if (JOYSTICK_SCALING) {
                    lsy=Math.pow(lsy,5.0);
                    lsx=Math.pow(lsx,5.0);
                    rsx=Math.pow(rsx,5.0);
                }

                double r = Math.sqrt(lsy*lsy+lsx*lsx);
                double robotAngle = Math.atan2(-1*lsx,lsy) - Math.PI / 4;
                double rightX = -1 * rsx;
                final double v1 = r * Math.cos(robotAngle) + rightX;
                final double v2 = r * Math.sin(robotAngle) - rightX;
                final double v3 = r * Math.sin(robotAngle) + rightX;
                final double v4 = r * Math.cos(robotAngle) - rightX;

                double comp1=r*Math.cos(robotAngle); //look at hypothesis that turn component overwhelms vector component
                double comp2=rightX;

                telemetry.addData("Speed/Angle","Speed=%.2f Angle=%.2f",r,robotAngle);
                telemetry.addData("Comp1/Comp2","Comp1=%.2f Comp2=%.2f",comp1,comp2);

                lf.setPower(v1);
                lb.setPower(v2); //was rf
                rf.setPower(v3); //was lb
                rb.setPower(v4);

                double vpower = gamepad2.right_stick_y;
                double rpower = gamepad2.right_stick_x;

                //vacuum.setPower(vpower);
                //vacuumRelease.setPower(rpower);
            }

            int lfpos=lf.getCurrentPosition(); //show positions to help with auto mode
            int rfpos=rf.getCurrentPosition();
            int lbpos=lb.getCurrentPosition();
            int rbpos=rb.getCurrentPosition();

            telemetry.addData("Positions","lf=%d rf=%d lb=%d rb=%d",lfpos,rfpos,lbpos,rbpos);

            // Show the elapsed game time
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
