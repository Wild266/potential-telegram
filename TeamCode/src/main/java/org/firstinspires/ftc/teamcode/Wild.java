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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *///<-------- Initialization/telemetry comments

@TeleOp(name="Wild", group="Linear Opmode")
public class Wild extends LinearOpMode {

    //VuMarks
    VuforiaLocalizer vuforia;

    // Declare OpMode members.
    NormalizedColorSensor colorSensor;
    float[] hsvValues = new float[3];
    final float values[] = hsvValues;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lf=null;
    private DcMotor rf=null;
    private DcMotor lb=null;
    private DcMotor rb=null;

    //private DcMotor vacuum=null;
    //private DcMotor vacuumRelease=null; //this will be eliminated or change to standard servo

    private static boolean SHOW_CAMERA=true; //whether to show the camera on the phone screen
    private static boolean JOYSTICK_SCALING=true; //whether to scale joystick values by cubing value (more precision for small movements)

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
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        lf  = hardwareMap.get(DcMotor.class, "lf");
        rf  = hardwareMap.get(DcMotor.class, "rf");
        lb  = hardwareMap.get(DcMotor.class, "lb");
        rb  = hardwareMap.get(DcMotor.class, "rb");

        //vacuum = hardwareMap.get(DcMotor.class, "vacuum");
        //vacuumRelease  = hardwareMap.get(DcMotor.class, "release");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);

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
            if (gamepad1.x) {
                if (colorSensor instanceof SwitchableLight) {
                    ((SwitchableLight) colorSensor).enableLight(true);
                }
                NormalizedRGBA colors = colorSensor.getNormalizedColors();
                Color.colorToHSV(colors.toColor(), hsvValues);
                telemetry.addLine()
                        .addData("H", "%.3f", hsvValues[0])
                        .addData("S", "%.3f", hsvValues[1])
                        .addData("V", "%.3f", hsvValues[2]);
                telemetry.addLine()
                        .addData("a", "%.3f", colors.alpha)
                        .addData("r", "%.3f", colors.red)
                        .addData("g", "%.3f", colors.green)
                        .addData("b", "%.3f", colors.blue);

                int color = colors.toColor();
                telemetry.addLine("raw Android color: ")
                        .addData("a", "%02x", Color.alpha(color))
                        .addData("r", "%02x", Color.red(color))
                        .addData("g", "%02x", Color.green(color))
                        .addData("b", "%02x", Color.blue(color));

                float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
                colors.red /= max;
                colors.green /= max;
                colors.blue /= max;
                color = colors.toColor();

                telemetry.addLine("normalized color:  ")
                        .addData("a", "%02x", Color.alpha(color))
                        .addData("r", "%02x", Color.red(color))
                        .addData("g", "%02x", Color.green(color))
                        .addData("b", "%02x", Color.blue(color));
                //telemetry.update();

                Color.RGBToHSV(Color.red(color), Color.green(color), Color.blue(color), hsvValues);
                }

            if (gamepad1.y) { //stop and look for vumarks if Y key is down

                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                    telemetry.addData("VuMark", "%s visible", vuMark);
                }

            } else {
                if (gamepad1.a) { //attempt to switch lefty and righty controls
                    double lsy=gamepad1.left_stick_y;
                    double lsx=gamepad1.left_stick_x;//right stick drive, left stick rotation
                    double rsx=gamepad1.right_stick_x;//@above ideal for righties

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
                    rf.setPower(v2);
                    lb.setPower(v3);
                    rb.setPower(v4);

                    double vpower = gamepad2.right_stick_y;
                    double rpower = gamepad2.right_stick_x;

                }  //the above is optionally executed if the button is pressed but so is the regular control below -- perhaps you meant the below to be an "else" block???
                double lsy=gamepad1.right_stick_y;
                double lsx=gamepad1.right_stick_x;//right stick drive, left stick rotation
                double rsx=gamepad1.left_stick_x;//@above ideal for righties

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
                rf.setPower(v2);
                lb.setPower(v3);
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
    } //<---- program
}//<----- Ending/Closing bracket Yee!!!!!!!!!!!!!!!1
