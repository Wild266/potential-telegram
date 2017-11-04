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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Team8235 TeleOp OpMode
 */

//SENSORS
// gyro is at 20
// color sensor at 48
// color sensor 3C default address for one on the arm

//SERVOS
// channel 5 left gripper   0=in 220=out
// channel 4 right gripper  0=out 220=in
// channel 1 relic arm      0=out 220=in
// channel 6 ball arm       0=down 150=up

//MOTORS
// controller OPQ5 motor1(lf) positive=forward
// controller OPQ5 motor2(rf) postiive=forward

// controller QHLS motor1(lb) positive=forward
// controller QHLS motor2(rb??)  DOES NOTHING/SOMETHING WRONG?

// controller VKGI motor1(vacuum pump)
// controller VKGI motor2(relic arm up) positive=up negative=down

// controller VUZO motor1(linear slide up) positive=in negative=out
// controller VUZO motor2(gripper lift) positive=down negative=up -- gravity wins

//PROBLEMS
// rf running backwards (we reversed initial position here)
// rb doesn't move (IS REVERSED) (fixed through reversing with other motor on controller)
// jewel arm (up and down) doesn't work
// opposite on left trigger
// top hat up and down ???



@TeleOp(name = "JavaTeleOp", group = "Linear Opmode")
public class Team8535JavaTeleOp extends LinearOpMode {

    private boolean prodbot = false;

    //Speed Factor for Fast/Slow Mode
    private double speedFactor = 1.0; //default full speed

    private ElapsedTime runtime = new ElapsedTime();
    private double vacuumTime=0.0;

    //Drive Motors
    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;

    //Front Gripper
    private DcMotor gripperLiftMotor = null;
    private Servo gripperLeftServo = null;
    private Servo gripperRightServo = null;

    private int heightPosition1 = 0; //probably too low but need calibration
    private int heightPosition2 = 100;
    private int heightPosition3 = 200;
    private int heightPosition4 = 300;

    //Relic Arm
    private DcMotor vacuumMotor = null;
    private DcMotor armExtendMotor = null;
    private DcMotor armLiftMotor = null;
    private Servo relicLiftServo = null;
    private Servo vacuumReleaseServo = null;

    //Ball Arm
    private Servo ballArmServo = null;
    private ColorSensor ballColorSensor = null;

    //State Variables
    private boolean vacuumRunning=false;

    private static boolean JOYSTICK_SCALING = true; //whether to scale joystick values by cubing value (more precision for small movements)

    //introduce methods which allow us to continue even when particular devices aren't found (by just not utilizing those devices)

    private DcMotor getMotor(String motorName) { //these could be made generic using type notation
        try {
            return (hardwareMap.get(DcMotor.class, motorName));
        } catch (Exception e) {
            return (null);
        }
    }

    private I2cDevice getDevice(String deviceName) { //these could be made generic using type notation
        try {
            return (hardwareMap.get(I2cDevice.class, deviceName));
        } catch (Exception e) {
            return (null);
        }
    }

    private Servo getServo(String servoName) { //these could be made generic using type notation
        try {
            return (hardwareMap.get(Servo.class, servoName));
        } catch (Exception e) {
            return (null);
        }
    }

    private ColorSensor getColorSensor(String sensorName) { //these could be made generic using type notation
        try {
            return (hardwareMap.get(ColorSensor.class, sensorName));
        } catch (Exception e) {
            return (null);
        }
    }

    /**
     * Set motors for a mecanum move
     * @param lsx left stick x (left/right)
     * @param lsy left stick y (front/back)
     * @param rsx right stick x (rotation)
     */
    private void mecanumMove(double lsx,double lsy,double rsx) {

        if (JOYSTICK_SCALING) {
            lsy=Math.pow(lsy,5.0);
            lsx=Math.pow(lsx,5.0);
            rsx=Math.pow(rsx,5.0);
        }

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

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        if (prodbot) {
            telemetry.addData("Bot", "ProdBot");
        } else {
            telemetry.addData("Bot", "DriveBot");
        }
        telemetry.update();

        //initialize required motors

        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");

        //initialize motors/servos/sensors that may vary between bot versions

        gripperLiftMotor = getMotor("gripper_lift");
        gripperLeftServo = getServo("gripper_left");
        gripperRightServo = getServo("gripper_right");
        vacuumMotor = getMotor("vacuum");
        armExtendMotor = getMotor("arm_extend");
        armLiftMotor = getMotor("arm_lift");
        relicLiftServo = getServo("relic_lift");
        vacuumReleaseServo = getServo("vacuum_release");
        ballArmServo = getServo("ball_arm");
        ballColorSensor = getColorSensor("ball_color");
    // bottom color sensor need to put in
    // gyro sensor need to put in

        if (gripperLeftServo!=null) gripperLeftServo.scaleRange(0.0,1.0); //tune these later to desired range
        if (gripperRightServo!=null) gripperRightServo.scaleRange(0.0,1.0);
        if (vacuumReleaseServo!=null) vacuumReleaseServo.scaleRange(0.0,1.0);
        if (ballArmServo!=null) ballArmServo.scaleRange(0.0,1.0);

        if (gripperLeftServo!=null) gripperLeftServo.setDirection(Servo.Direction.FORWARD);
        if (gripperRightServo!=null) gripperRightServo.setDirection(Servo.Direction.FORWARD);
        if (vacuumReleaseServo!=null) vacuumReleaseServo.setDirection(Servo.Direction.FORWARD);
        if (ballArmServo!=null) ballArmServo.setDirection(Servo.Direction.FORWARD);

        if (getDevice("drivebot") != null) {
            prodbot = false;
        } else {
            prodbot = true;
        }
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        if (prodbot) {
            lf.setDirection(DcMotor.Direction.REVERSE); //was REVERSE
            rf.setDirection(DcMotor.Direction.REVERSE); //was REVERSE
            lb.setDirection(DcMotor.Direction.REVERSE); //was FORWARD
            rb.setDirection(DcMotor.Direction.FORWARD); //was FORWARD

            if (gripperLiftMotor!=null) gripperLiftMotor.setDirection(DcMotor.Direction.FORWARD);
            if (vacuumMotor!=null) vacuumMotor.setDirection(DcMotor.Direction.FORWARD);
            if (armExtendMotor!=null) armExtendMotor.setDirection(DcMotor.Direction.FORWARD);
            if (armLiftMotor!=null) armLiftMotor.setDirection(DcMotor.Direction.FORWARD);

        } else {
            lf.setDirection(DcMotor.Direction.REVERSE); //was REVERSE
            rf.setDirection(DcMotor.Direction.FORWARD); //was REVERSE
            lb.setDirection(DcMotor.Direction.REVERSE); //was FORWARD
            rb.setDirection(DcMotor.Direction.FORWARD); //was FORWARD

            if (gripperLiftMotor!=null) gripperLiftMotor.setDirection(DcMotor.Direction.FORWARD);
            if (vacuumMotor!=null) vacuumMotor.setDirection(DcMotor.Direction.FORWARD);
            if (armExtendMotor!=null) armExtendMotor.setDirection(DcMotor.Direction.FORWARD);
            if (armLiftMotor!=null) armLiftMotor.setDirection(DcMotor.Direction.FORWARD);
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoders
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (gripperLiftMotor!=null) {
            gripperLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            gripperLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //we will probably switch modes for this motor for presets
        }

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //runs again
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (ballColorSensor!=null) {
            ballColorSensor.enableLed(true);
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //translate some gamepad controls to variables

            //gamepad1
            double forwardBack = gamepad1.left_stick_y;
            double leftRight = gamepad1.left_stick_x;
            double ccwCwRotate = gamepad1.right_stick_x;
            boolean slowMode = (gamepad1.left_trigger > 0.0);
            boolean lowerBallArm = (gamepad1.dpad_down);
            boolean raiseBallArm = (gamepad1.dpad_up);
            double vacuumRelease = gamepad1.right_trigger; //couldn't find room on gamepad2

            //gamepad2
            double raiseLowerLift = gamepad2.left_stick_y;
            double leftClamp = gamepad2.left_trigger;
            double rightClamp = gamepad2.right_trigger;
            boolean height1 = gamepad2.x; //lowest preset gripper height
            boolean height2 = gamepad2.a;
            boolean height3 = gamepad2.b;
            boolean height4 = gamepad2.y; //highest preset gripper height
            boolean extendRelicArm = gamepad2.dpad_right;
            boolean retractRelicArm = gamepad2.dpad_left;
            boolean raiseRelic = gamepad2.dpad_up;
            boolean lowerRelic = gamepad2.dpad_down;
            double lowerRaiseArm = gamepad2.right_stick_y;
            boolean startVacuum = gamepad2.start;
            boolean stopVacuum = gamepad2.back;

            if (slowMode) {
                speedFactor = 0.75;
            } else {
                speedFactor = 1.0;
            }

            double lsy = forwardBack * speedFactor;
            double lsx = leftRight * speedFactor;
            double rsx = ccwCwRotate * (speedFactor * 0.85);

            //this takes care of the movement commands by calculating use of the mecanum wheels
            mecanumMove(lsx,lsy,rsx);

            if (gripperLiftMotor!=null) {
                if (raiseLowerLift!=0.0) {
                    if (gripperLiftMotor.getMode()!=DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                        gripperLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    gripperLiftMotor.setPower(raiseLowerLift);
                } else {
                    if (height1 || height2 || height3 || height4) {
                        //we're assuming the encoder was zeroed at the bottom before the op mode started (and has tracked changes)
                        if (gripperLiftMotor.getMode()!=DcMotor.RunMode.RUN_TO_POSITION) {
                            gripperLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        }
                        if (height1) gripperLiftMotor.setTargetPosition(heightPosition1);
                        if (height2) gripperLiftMotor.setTargetPosition(heightPosition2);
                        if (height3) gripperLiftMotor.setTargetPosition(heightPosition3);
                        if (height4) gripperLiftMotor.setTargetPosition(heightPosition4);
                        gripperLiftMotor.setPower(1.0);

                        while (opModeIsActive() && gripperLiftMotor.isBusy()) { //this freezes until it's in position -- might want to do this during other things
                            telemetry.addData("LiftPosition",  "Running To %7d At %7d",gripperLiftMotor.getTargetPosition(),gripperLiftMotor.getCurrentPosition());
                            telemetry.update();
                        }

                        gripperLiftMotor.setPower(0);
                        gripperLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    }
                }
            }

            if (vacuumMotor!=null) {
                if (startVacuum && !vacuumRunning && ((runtime.time()-vacuumTime)>0.2)) {
                    vacuumRunning=true;
                    vacuumTime=runtime.time();
                } else if (startVacuum && vacuumRunning && ((runtime.time()-vacuumTime)>0.2)) {
                    vacuumRunning=false;
                    vacuumTime=runtime.time();
                } else if (startVacuum) {
                    vacuumTime=runtime.time(); //to avoid self-toggle
                }
                if (stopVacuum) vacuumRunning=false; //can't get back button to work
                if (vacuumRunning) {
                    telemetry.addData("Vacuum","On");
                    vacuumMotor.setPower(1.0);
                } else {
                    telemetry.addData("Vacuum","Off");
                    vacuumMotor.setPower(0.0);
                }
            }
            if (armExtendMotor!=null) {
                if (extendRelicArm && !retractRelicArm) armExtendMotor.setPower(1.0);
                if (retractRelicArm && !extendRelicArm) armExtendMotor.setPower(-1.0);
            }

            if (relicLiftServo!=null) {
                if (raiseRelic && !lowerRelic) relicLiftServo.setPosition(1.0);
                if (lowerRelic && !raiseRelic) relicLiftServo.setPosition(0.0);
            }

            if (armLiftMotor!=null) {
                armLiftMotor.setPower(raiseLowerLift);
            }

            if (gripperLeftServo!=null) {
                gripperLeftServo.setPosition(leftClamp);
            }

            if (gripperRightServo!=null) {
                gripperRightServo.setPosition(rightClamp);
            }

            if (vacuumReleaseServo !=null) {
                vacuumReleaseServo.setPosition(vacuumRelease);
            }

            if (ballArmServo !=null) {
                if (lowerBallArm && !raiseBallArm) ballArmServo.setPosition(1.0);
                if (raiseBallArm && !lowerBallArm) ballArmServo.setPosition(0);
            }

            if (ballColorSensor!=null)
                telemetry.addData("BallColor","R=%d G=%d B=%d A=%d", ballColorSensor.red(), ballColorSensor.green(), ballColorSensor.blue(), ballColorSensor.alpha());

            int lfpos = lf.getCurrentPosition(); //show positions to help with auto mode
            int rfpos = rf.getCurrentPosition();
            int lbpos = lb.getCurrentPosition();
            int rbpos = rb.getCurrentPosition();

            telemetry.addData("Positions", "lf=%d rf=%d lb=%d rb=%d", lfpos, rfpos, lbpos, rbpos);

            // Show the elapsed game time
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
