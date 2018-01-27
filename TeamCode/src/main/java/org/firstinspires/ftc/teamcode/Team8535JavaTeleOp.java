/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 *
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
    private double vacuumTime = 0.0;
    private double vacuumTime2 = 0.0; //added for second vacuum pump
    private double lastLoopTime = 0.0;
    private double currentLoopTime = 0.0;

    //Drive Motors
    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;


    //Front Gripper
    private DcMotor gripperLiftMotor = null;
    private Servo gripperTwistServo = null;
    private double gripperTwistPosition = 1.0;
    private double gripperTwistSpeed = 2.0;
    /*private Servo gripperLeftServo = null;
    private Servo gripperRightServo = null;

    private double gripperLeftPosition = 0.5; //initial position of left gripper (tune this)
    private double gripperRightPosition = 0.5; //initial position of right gripper (tune this)
    private double gripperLeftClosingSpeed = 4.0; //range per second
    private double gripperRightClosingSpeed = 4.0; //range per second
    private double gripperLeftOpeningSpeed = 5.0; //range per second
    private double gripperRightOpeningSpeed = 5.0; //range per second

    private int heightPosition1 = 0; //probably too low but need calibration
    private int heightPosition2 = 100;
    private int heightPosition3 = 200;
    private int heightPosition4 = 300;
*/
    //Relic Arm
    private DcMotor vacuumMotor = null;
    private DcMotor vacuumMotor2 = null; //added
    private DcMotor armExtendMotor = null;
    private Servo armLiftServo = null; //changed to servo
    private double armLiftPosition = 0.5;
    private Servo relicLiftServo = null;
    private Servo relicClawServo = null;
    private double relicClawPosition = 1.0;
    private double relicClawSpeed = 0.5;
    private Servo vacuumReleaseServo = null;
    private Servo vacuumReleaseServo2 = null; //added
    private double vacuumReleasePosition = 0.5;
    private double vacuumReleasePosition2 = 0.5; //changed to 0.5 (why 0.7?)

    //private double vacuumReleasePosition3 = 0.3;    //As: for the new vacuums
    private double vacuumReleaseSpeed = 3.0;

    private double relicLiftPosition = 1.0; //initial position (tune this)
    private double relicLiftSpeed = 1.0;
    private double armLiftSpeed = 0.5;

    //Block Tilt
    private Servo blockTiltServo = null;
    private double blockTiltPosition = 0.95;
    private double blockTiltSpeed = 0.7;

    //Ball Arm
    private Servo ballArmServo = null;
    private ColorSensor ballColorSensor = null;

    private double ballArmPosition = 0.0;//initial position of ball arm servo (tune this)
    private double ballArmSpeed = 0.5; //range per second
    //Base
    private ColorSensor bottomColorSensor = null;

    private ModernRoboticsI2cGyro gyro = null;

    //State Variables
    private boolean vacuumRunning = false;
    private boolean vacuumRunning2 = false; //added

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

    private ModernRoboticsI2cGyro getGyro(String gyroName) { //these could be made generic using type notation
        try {
            return (hardwareMap.get(ModernRoboticsI2cGyro.class, gyroName));
        } catch (Exception e) {
            return (null);
        }
    }

    /**
     * Set motors for a mecanum move
     *
     * @param lsx left stick x (left/right)
     * @param lsy left stick y (front/back)
     * @param rsx right stick x (rotation)
     */
    private void mecanumMove(double lsx, double lsy, double rsx) {

        if (JOYSTICK_SCALING) {
            lsy = Math.pow(lsy, 1.0);
            lsx = Math.pow(lsx, 1.0);
            rsx = Math.pow(rsx, 1.0);
        }

        double r = Math.sqrt(lsy * lsy + lsx * lsx);
        double robotAngle = Math.atan2(-1 * lsy, lsx) - Math.PI / 4;
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
        if (getDevice("drivebot") != null) {
            prodbot = false;
            telemetry.addData("Bot", "DriveBot");
        } else {
            prodbot = true;
            telemetry.addData("Bot", "ProdBot");
        }
        telemetry.update();

        //initialize required motors

        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");

        //initialize motors/servos/sensors that may vary between bot versions

        gripperLiftMotor = getMotor("gripper_lift");  //moves the gripper assembly in the front of the bot up and down
        if (gripperLiftMotor != null) gripperLiftMotor.setPower(0.0);
        //gripperLeftServo = getServo("gripper_left");
        //if (gripperLeftServo != null) gripperLeftServo.setPosition(gripperLeftPosition);
        //gripperRightServo = getServo("gripper_right");
        //if (gripperRightServo != null) gripperRightServo.setPosition(gripperRightPosition);
        vacuumMotor = getMotor("vacuum"); //vacuum 1
        if (vacuumMotor != null) vacuumMotor.setPower(0.0);
        vacuumMotor2 = getMotor("vacuum2"); //vacuum 2
        if (vacuumMotor2 != null) vacuumMotor2.setPower(0.0);
        armExtendMotor = getMotor("arm_extend"); //moves the relic arm linear slide out and back
        if (armExtendMotor != null) armExtendMotor.setPower(0.0);
        armLiftServo = getServo("arm_lift"); //lifts the rack and pinion for the relic arm up and down
        if (armLiftServo != null) armLiftServo.setPosition(armLiftPosition);
        relicLiftServo = getServo("relic_lift"); //lift the relic gripper (not the claw) up and down
        if (relicLiftServo != null) relicLiftServo.setPosition(relicLiftPosition);
        gripperTwistServo = getServo("gripper_twist"); //rotates the suction cup arm for the front gripper
        if (gripperTwistServo !=null) gripperTwistServo.setPosition(gripperTwistPosition);
        relicClawServo = getServo ("relic_claw"); //clamp the claw over the relic
        if (relicClawServo != null) relicClawServo.setPosition(relicClawPosition);
        blockTiltServo = getServo ("block_tilt"); //tilt the block holder on top of the bot
        if (blockTiltServo != null) blockTiltServo.setPosition(blockTiltPosition);
        vacuumReleaseServo = getServo("vacuum_release"); //release vacuum 1
        if (vacuumReleaseServo != null) vacuumReleaseServo.setPosition(vacuumReleasePosition);
        vacuumReleaseServo2 = getServo("vacuum_release2"); //release vacuum 2
        if (vacuumReleaseServo2 != null) vacuumReleaseServo2.setPosition(vacuumReleasePosition2);
        ballArmServo = getServo("ball_arm"); //raise/lower ball arm
        if (ballArmServo != null) ballArmServo.setPosition(ballArmPosition);
        ballColorSensor = getColorSensor("ball_color");
        bottomColorSensor = getColorSensor("bottom_color");
        if (bottomColorSensor != null)
            bottomColorSensor.setI2cAddress(I2cAddr.create7bit(0x48)); //we believe these are 7bit addresses
        gyro = getGyro("gyro");
        if (gyro != null) { //just rename the gyro in the resource file to run without it
            gyro.setI2cAddress(I2cAddr.create7bit(0x10)); //we believe these are 7bit addresses
            telemetry.log().add("Gyro Calibrating. Do Not Move!");
            gyro.calibrate();
            // Wait until the gyro calibration is complete
            runtime.reset();
            while (!isStopRequested() && gyro.isCalibrating()) {
                telemetry.addData("Calibrating", "%s", Math.round(runtime.seconds()) % 2 == 0 ? "|.." : "..|");
                telemetry.update();
                sleep(50);
            }
            telemetry.log().clear();
            telemetry.log().add("Gyro Calibrated. Press Start.");
            telemetry.clear();
            telemetry.update();
            gyro.resetZAxisIntegrator();
        }
        //if (gripperLeftServo!=null) gripperLeftServo.scaleRange(0.0,1.0); //tune these later to desired range
        //if (gripperRightServo!=null) gripperRightServo.scaleRange(0.0,1.0);
        if (vacuumReleaseServo != null) vacuumReleaseServo.scaleRange(0.0, 1.0);
        if (vacuumReleaseServo2 != null) vacuumReleaseServo2.scaleRange(0.0, 1.0);
        if (ballArmServo != null) ballArmServo.scaleRange(0.0, 1.0);

        //if (gripperLeftServo!=null) gripperLeftServo.setDirection(Servo.Direction.REVERSE); //changed to reverse to flip left gripper servo direction
        //if (gripperRightServo!=null) gripperRightServo.setDirection(Servo.Direction.FORWARD);
        if (vacuumReleaseServo != null) vacuumReleaseServo.setDirection(Servo.Direction.FORWARD);
        if (vacuumReleaseServo2 != null) vacuumReleaseServo2.setDirection(Servo.Direction.REVERSE);
        if (ballArmServo != null) ballArmServo.setDirection(Servo.Direction.FORWARD);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        if (prodbot) {
            lf.setDirection(DcMotor.Direction.REVERSE); //was REVERSE
            rf.setDirection(DcMotor.Direction.FORWARD); //was REVERSE
            lb.setDirection(DcMotor.Direction.REVERSE); //was FORWARD
            rb.setDirection(DcMotor.Direction.FORWARD); //was FORWARD

            if (gripperLiftMotor != null) gripperLiftMotor.setDirection(DcMotor.Direction.FORWARD);
            if (vacuumMotor != null) vacuumMotor.setDirection(DcMotor.Direction.FORWARD);
            if (vacuumMotor2 != null) vacuumMotor2.setDirection(DcMotor.Direction.FORWARD);
            if (armExtendMotor != null) armExtendMotor.setDirection(DcMotor.Direction.FORWARD);
            if (armLiftServo != null) armLiftServo.setDirection(Servo.Direction.FORWARD);

        } else {
            lf.setDirection(DcMotor.Direction.REVERSE); //was REVERSE
            rf.setDirection(DcMotor.Direction.FORWARD); //was REVERSE
            lb.setDirection(DcMotor.Direction.REVERSE); //was FORWARD
            rb.setDirection(DcMotor.Direction.FORWARD); //was FORWARD

            if (gripperLiftMotor != null) gripperLiftMotor.setDirection(DcMotor.Direction.FORWARD);
            if (vacuumMotor != null) vacuumMotor.setDirection(DcMotor.Direction.FORWARD);
            if (vacuumMotor2 != null) vacuumMotor2.setDirection(DcMotor.Direction.FORWARD);
            if (armExtendMotor != null) armExtendMotor.setDirection(DcMotor.Direction.FORWARD);
            if (armLiftServo != null) armLiftServo.setDirection(Servo.Direction.FORWARD);
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoders
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (gripperLiftMotor != null) {
            gripperLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            gripperLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //we will probably switch modes for this motor for presets
        }

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //runs again
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (ballColorSensor != null) {
            ballColorSensor.enableLed(true);
        }

        if (bottomColorSensor != null) {
            bottomColorSensor.enableLed(true);
        }

        currentLoopTime = runtime.time();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            lastLoopTime = currentLoopTime; //the loop timing difference gives our actively stepping servos the time to use with their speed multipliers
            currentLoopTime = runtime.time();
            telemetry.addData("Loop Time", (currentLoopTime - lastLoopTime)); //hopefully reasonable speed for movement determinations

            //translate some gamepad controls to variables

            //gamepad1
            double forwardBack = gamepad1.left_stick_y;
            double leftRight = gamepad1.left_stick_x;
            double ccwCwRotate = gamepad1.right_stick_x;
            boolean slowMode = (gamepad1.left_trigger > 0.2);
            boolean lowerBallArm = (gamepad1.dpad_down);
            boolean raiseBallArm = (gamepad1.dpad_up);
            //double vacuumRelease = gamepad1.right_trigger; //couldn't find room on gamepad2
            //boolean vacuumRelease2 = gamepad1.a;            //As: will modify while testing
            //boolean vacuumRelease3 = gamepad1.b;            //As: will modify while testing
            //boolean vacuumClose = gamepad1.right_bumper;

            //gamepad2
            double raiseLowerLift = gamepad2.left_stick_y;
            double leftClamp = gamepad2.left_trigger;
            double rightClamp = gamepad2.right_trigger;
            boolean leftRelease = gamepad2.left_bumper;
            boolean rightRelease = gamepad2.right_bumper;
            /*
            boolean height1 = gamepad2.x; //lowest preset gripper height
            boolean height2 = gamepad2.a;
            boolean height3 = gamepad2.b;
            boolean height4 = gamepad2.y; //highest preset gripper height
            */
            boolean extendRelicArm = gamepad2.dpad_right;
            boolean retractRelicArm = gamepad2.dpad_left;
            //boolean raiseRelic = gamepad2.dpad_up; //was raiseRelic
            //boolean lowerRelic = gamepad2.dpad_down; //was lowerRelic
            //double lowerRaiseArm = gamepad2.right_stick_y;  //
            boolean raiseRelicArm = gamepad2.dpad_up;
            boolean lowerRelicArm = gamepad2.dpad_down;
            boolean raiseRelic = gamepad2.y;//gamepad2.right_stick_y>0.2;
            boolean lowerRelic = gamepad2.a;//gamepad2.right_stick_y<-0.2;

            //new vacuum controls
            boolean toggleVacuumBoth = gamepad2.start; //toggle both vacuum pumps together
            boolean toggleVacuum1 = gamepad2.left_bumper; //toggle pump1
            boolean toggleVacuum2 = gamepad2.right_bumper; //toggle pump2
            double vacuumRelease1 = gamepad2.left_trigger; //release pump1
            double vacuumRelease2 = gamepad2.right_trigger; //release pump2
            double gripperTwistCW = gamepad2.right_stick_x; //was left_stick_x
            boolean relicClawOpen = gamepad2.x; //was clawOpen
            boolean relicClawClose = gamepad2.b; //was clawClose
            boolean blockTiltUp = gamepad1.y;
            boolean blockTiltDown = gamepad1.a;
            //boolean stopVacuum = gamepad2.back;

            if (slowMode) {
                speedFactor = 0.9; //was 0.75;
            } else {
                speedFactor = 1.0;
            }

            double lsy = forwardBack * speedFactor;
            double lsx = leftRight * speedFactor;
            double rsx = ccwCwRotate;
            if (speedFactor < 1.0) {
                rsx *= 0.8;
            }
            //this takes care of the movement commands by calculating use of the mecanum wheels
            mecanumMove(lsx, lsy, rsx);

            if (gripperLiftMotor != null) {
                if (raiseLowerLift < -0.09/*-0.2*/) {
                    gripperLiftMotor.setPower(raiseLowerLift/*-1.0*/); //if trigger is positive, raise lift
                    telemetry.addData("Gripper Lift", "Raising");
                } else if (raiseLowerLift > 0.09/*0.2*/) {
                    gripperLiftMotor.setPower(raiseLowerLift/*1.0*/); //if trigger is negative, lower lift
                    telemetry.addData("Gripper Lift", "Lowering");
                } else {
                    gripperLiftMotor.setPower(0.0); //this should do motor braking
                    telemetry.addData("Gripper Lift", "Stopped");
                }

                /* commenting this out until at least the above manual movement works
                if (height1 || height2 || height3 || height4) {
                    //we're assuming the encoder was zeroed at the bottom before the op mode started (and has tracked changes)
                    if (gripperLiftMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                        gripperLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    if (height1) gripperLiftMotor.setTargetPosition(heightPosition1);
                    if (height2) gripperLiftMotor.setTargetPosition(heightPosition2);
                    if (height3) gripperLiftMotor.setTargetPosition(heightPosition3);
                    if (height4) gripperLiftMotor.setTargetPosition(heightPosition4);
                    gripperLiftMotor.setPower(1.0);

                    while (opModeIsActive() && gripperLiftMotor.isBusy()) { //this freezes until it's in position -- might want to do this during other things
                        telemetry.addData("LiftPosition", "Running To %7d At %7d", gripperLiftMotor.getTargetPosition(), gripperLiftMotor.getCurrentPosition());
                        telemetry.update();
                    }

                    gripperLiftMotor.setPower(0);
                    gripperLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                */

                if (toggleVacuumBoth) { //for dual toggle
                    toggleVacuum1 = true;
                    toggleVacuum2 = true;
                }
                if (vacuumMotor != null) {
                    if (toggleVacuum1 && !vacuumRunning && ((runtime.time() - vacuumTime) > 0.2)) {
                        vacuumRunning = true;
                        vacuumTime = runtime.time();
                    } else if (toggleVacuum1 && vacuumRunning && ((runtime.time() - vacuumTime) > 0.2)) {
                        vacuumRunning = false;
                        vacuumTime = runtime.time();
                    } else if (toggleVacuum1) {
                        vacuumTime = runtime.time(); //to avoid self-toggle
                    }
                    //if (stopVacuum) vacuumRunning=false; //can't get back button to work
                    if (vacuumRunning) {
                        telemetry.addData("Vacuum1", "On");
                        vacuumMotor.setPower(1.0);
                    } else {
                        telemetry.addData("Vacuum1", "Off");
                        vacuumMotor.setPower(0.0);
                    }
                }

                if (vacuumMotor2 != null) {
                    if (toggleVacuum2 && !vacuumRunning2 && ((runtime.time() - vacuumTime2) > 0.2)) {
                        vacuumRunning2 = true;
                        vacuumTime2 = runtime.time();
                    } else if (toggleVacuum2 && vacuumRunning2 && ((runtime.time() - vacuumTime2) > 0.2)) {
                        vacuumRunning2 = false;
                        vacuumTime2 = runtime.time();
                    } else if (toggleVacuum2) {
                        vacuumTime2 = runtime.time(); //to avoid self-toggle
                    }
                    //if (stopVacuum) vacuumRunning=false; //can't get back button to work
                    if (vacuumRunning2) {
                        telemetry.addData("Vacuum2", "On");
                        vacuumMotor2.setPower(1.0);
                    } else {
                        telemetry.addData("Vacuum2", "Off");
                        vacuumMotor2.setPower(0.0);
                    }
                }

                if (armExtendMotor != null) {
                    if (extendRelicArm && !retractRelicArm) {
                        armExtendMotor.setPower(0.8);
                    } else if (retractRelicArm && !extendRelicArm) {
                        armExtendMotor.setPower(-0.8);
                    } else {
                        armExtendMotor.setPower(0.0);
                    }
                }

                if (relicLiftServo != null) {
                    if (raiseRelic) { //step it up
                        relicLiftPosition += relicLiftSpeed * (currentLoopTime - lastLoopTime);
                        if (relicLiftPosition > 1.0) relicLiftPosition = 1.0;
                    } else if (lowerRelic) { //step it down
                        relicLiftPosition -= relicLiftSpeed * (currentLoopTime - lastLoopTime);
                        if (relicLiftPosition < 0.0) relicLiftPosition = 0.0;
                    }
                    relicLiftServo.setPosition(relicLiftPosition);
                    telemetry.addData("Relic Lift", relicLiftPosition);
                }

                //if (armLiftServo!=null) {
                //    if (lowerRaiseArm > 0.2) {
                //      armLiftServo.setPower(1.0); //if trigger is positive, raise arm
                // } else if (lowerRaiseArm < -0.2) {
                //   armLiftServo.setPower(-1.0); //if trigger is negative, lower rm
                //} else {
                //  armLiftServo.setPower(0.0); //this should do motor braking
                //}


                if (armLiftServo != null) {
                    if (raiseRelicArm) {
                        armLiftPosition -= armLiftSpeed * (currentLoopTime - lastLoopTime);
                        if (armLiftPosition < 0.0) armLiftPosition = 0.0;
                    } else if (lowerRelicArm) {
                        armLiftPosition += armLiftSpeed * (currentLoopTime - lastLoopTime);
                        if (armLiftPosition > 1.0) armLiftPosition = 1.0;
                    } else {
                        armLiftPosition=0.5;
                    }
                    armLiftServo.setPosition(armLiftPosition);
                    telemetry.addData("Arm Lift", armLiftPosition);

                }

                if (gripperTwistServo != null) {
                    if (gripperTwistCW > 0.2) {
                        gripperTwistPosition += gripperTwistSpeed * (currentLoopTime - lastLoopTime);
                        if (gripperTwistPosition > 1.0) gripperTwistPosition = 1.0;
                    } else if (gripperTwistCW < -0.2) {
                        gripperTwistPosition -= gripperTwistSpeed * (currentLoopTime - lastLoopTime);
                        if (gripperTwistPosition < 0.0) gripperTwistPosition = 0.0;
                    }
                    gripperTwistServo.setPosition(gripperTwistPosition);
                    telemetry.addData("Gripper Twist", gripperTwistPosition);

                }

                if (relicClawServo != null) {
                    if (relicClawOpen) {
                        relicClawPosition += relicClawSpeed * (currentLoopTime - lastLoopTime);
                        if (relicClawPosition > 1.0) relicClawPosition = 1.0;
                    } else if (relicClawClose) {
                        relicClawPosition -= relicClawSpeed * (currentLoopTime - lastLoopTime);
                        if (relicClawPosition < 0.0) relicClawPosition = 0.0;
                    }else {
                        relicClawPosition=0.5;
                    }
                    relicClawServo.setPosition(relicClawPosition);
                    telemetry.addData("Relic Claw", relicClawPosition);

                }

                if (blockTiltServo != null) {
                    if (blockTiltUp) {
                        blockTiltPosition += blockTiltSpeed * (currentLoopTime - lastLoopTime);
                        if (blockTiltPosition > 1.0) blockTiltPosition = 1.0;
                    } else if (blockTiltDown) {
                        blockTiltPosition -= blockTiltSpeed * (currentLoopTime - lastLoopTime);
                        if (blockTiltPosition < 0.0) blockTiltPosition = 0.0;
                    }
                    blockTiltServo.setPosition(blockTiltPosition);
                    telemetry.addData("Block Tilt", blockTiltPosition);

                }


            /*if (gripperLeftServo!=null) {
                if (leftClamp>0.2) { //step it up
                    gripperLeftPosition+=gripperLeftClosingSpeed*(currentLoopTime-lastLoopTime);
                    if (gripperLeftPosition>1.0) gripperLeftPosition=1.0;
                } else if (leftRelease) { //step it down
                    gripperLeftPosition-=gripperLeftOpeningSpeed*(currentLoopTime-lastLoopTime);
                    if (gripperLeftPosition<0.0) gripperLeftPosition=0.0;
                }
                gripperLeftServo.setPosition(gripperLeftPosition);
                telemetry.addData("Left Gripper",gripperLeftPosition);
            }

            if (gripperRightServo!=null) {
                if (rightClamp>0.2) { //step it up
                    gripperRightPosition+=gripperRightClosingSpeed*(currentLoopTime-lastLoopTime);
                    if (gripperRightPosition>1.0) gripperRightPosition=1.0;
                } else if (rightRelease) { //step it down
                    gripperRightPosition-=gripperRightOpeningSpeed*(currentLoopTime-lastLoopTime);
                    if (gripperRightPosition<0.0) gripperRightPosition=0.0;
                }
                gripperRightServo.setPosition(gripperRightPosition);
                telemetry.addData("Right Gripper",gripperRightPosition);
            }
            */

                if (vacuumReleaseServo != null) {
                    if (vacuumRelease1>0.09) {
                        vacuumReleasePosition=0.0;
                    } else {
                        vacuumReleasePosition=1.0;
                    }
                    /*
                    if (vacuumRelease1 > 0.2) { //step it up
                        vacuumReleasePosition -= vacuumReleaseSpeed * (currentLoopTime - lastLoopTime);
                        if (vacuumReleasePosition < 0.0) vacuumReleasePosition = 0.0;
                    } else if (vacuumRelease1 < -0.2) { //step it down
                        vacuumReleasePosition += vacuumReleaseSpeed * (currentLoopTime - lastLoopTime);
                        if (vacuumReleasePosition > 1.0) vacuumReleasePosition = 1.0;
                    } else {
                        vacuumReleasePosition = 1.0; //probably want this for a continuous servo
                    }
                    */
                    vacuumReleaseServo.setPosition(vacuumReleasePosition);
                    telemetry.addData("Vacuum Release1", vacuumReleasePosition);
                }

                if (vacuumReleaseServo2 != null) {
                    if (vacuumRelease2>0.09) {
                        vacuumReleasePosition2=0.0;
                    } else {
                        vacuumReleasePosition2=1.0;
                    }
                    /*
                    if (vacuumRelease2 > 0.2) { //step it up
                        vacuumReleasePosition2 -= vacuumReleaseSpeed * (currentLoopTime - lastLoopTime);
                        if (vacuumReleasePosition2 < 0.0) vacuumReleasePosition2 = 0.0;
                    } else if (vacuumRelease2 < -0.2) { //step it down
                        vacuumReleasePosition2 += vacuumReleaseSpeed * (currentLoopTime - lastLoopTime);
                        if (vacuumReleasePosition2 > 1.0) vacuumReleasePosition2 = 1.0;
                    } else {
                        vacuumReleasePosition2 = 1.0; //probably want this for a continuous servo
                    }
                    */
                    vacuumReleaseServo2.setPosition(vacuumReleasePosition2);
                    telemetry.addData("Vacuum Release2", vacuumReleasePosition2);
                }

                if (ballArmServo != null) {
                    if (raiseBallArm) { //step it up
                        ballArmPosition += ballArmSpeed * (currentLoopTime - lastLoopTime);
                        if (ballArmPosition > 1.0) ballArmPosition = 1.0;
                    } else if (lowerBallArm) { //step it down
                        ballArmPosition -= ballArmSpeed * (currentLoopTime - lastLoopTime);
                        if (ballArmPosition < 0.0) ballArmPosition = 0.0;
                    } else {
                        ballArmPosition = 0.0; //probably want this for a continuous servo
                    }
                    ballArmServo.setPosition(ballArmPosition);
                    telemetry.addData("Ball Arm", ballArmPosition);
                }

                if (ballColorSensor != null)
                    telemetry.addData("BallColor", "R=%d G=%d B=%d A=%d", ballColorSensor.red(), ballColorSensor.green(), ballColorSensor.blue(), ballColorSensor.alpha());

                if (bottomColorSensor != null)
                    telemetry.addData("BottomColor", "R=%d G=%d B=%d A=%d", bottomColorSensor.red(), bottomColorSensor.green(), bottomColorSensor.blue(), bottomColorSensor.alpha());

                int lfpos = lf.getCurrentPosition(); //show positions to help with auto mode
                int rfpos = rf.getCurrentPosition();
                int lbpos = lb.getCurrentPosition();
                int rbpos = rb.getCurrentPosition();

                telemetry.addData("Positions", "lf=%d rf=%d lb=%d rb=%d", lfpos, rfpos, lbpos, rbpos);
                if (gyro != null)

                {
                    int heading = gyro.getHeading();
                    telemetry.addData("Heading", "%3d degrees", heading);
                }
                // Show the elapsed game time
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.update();
            }
        }
    }
}

