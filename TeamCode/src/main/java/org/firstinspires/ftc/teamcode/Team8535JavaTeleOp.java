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
    private int lastVac=0;
    private int curVac=0;
    private int lastVac2=0;
    private int curVac2=0;

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
    private double vacuumReleasePosition = 1.0; //changed to 1.0 from 0.5 on on 2/10
    private double vacuumReleasePosition2 = 1.0;

    private double vacuumReleaseSpeed = 3.0;

    private double relicLiftPosition = 1.0; //initial position (tune this)
    private double relicLiftSpeed = 0.5;
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
    private static boolean TELE = false; //show telemetry

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
        if (vacuumReleaseServo != null) vacuumReleaseServo.scaleRange(0.0, 1.0);
        if (vacuumReleaseServo2 != null) vacuumReleaseServo2.scaleRange(0.0, 1.0);
        if (ballArmServo != null) ballArmServo.scaleRange(0.0, 1.0);

        if (vacuumReleaseServo != null) vacuumReleaseServo.setDirection(Servo.Direction.FORWARD);
        if (vacuumReleaseServo2 != null) vacuumReleaseServo2.setDirection(Servo.Direction.REVERSE);
        if (ballArmServo != null) ballArmServo.setDirection(Servo.Direction.FORWARD);

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
        vacuumMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vacuumMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (gripperLiftMotor != null) {
            gripperLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            gripperLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //we will probably switch modes for this motor for presets
        }

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //runs again
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vacuumMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vacuumMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (ballColorSensor != null) {
            ballColorSensor.enableLed(true);
        }

        if (bottomColorSensor != null) {
            bottomColorSensor.enableLed(true);
        }

        currentLoopTime = runtime.time();
        //curVac=vacuumMotor.getCurrentPosition();
        //curVac2=vacuumMotor2.getCurrentPosition();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            lastLoopTime = currentLoopTime; //the loop timing difference gives our actively stepping servos the time to use with their speed multipliers
            //lastVac=curVac;
            //lastVac2=curVac2;
            currentLoopTime = runtime.time();
            //curVac=vacuumMotor.getCurrentPosition();
            //curVac2=vacuumMotor2.getCurrentPosition();
            //double timedif=(currentLoopTime-lastLoopTime);
            //telemetry.addData("Vac1 Speed RPM",60.0*(curVac-lastVac)/timedif);
            //telemetry.addData("Vac2 Speed RPM",60.0*(curVac2-lastVac2)/timedif);
            //telemetry.addData("Loop Time (Secs)",timedif); //hopefully reasonable speed for movement determinations

            //translate some gamepad controls to variables

            //gamepad1
            double forwardBack = gamepad1.left_stick_y;
            double leftRight = gamepad1.left_stick_x;
            double ccwCwRotate = gamepad1.right_stick_x;
            boolean slowMode = (gamepad1.left_trigger > 0.2);
            boolean lowerBallArm = (gamepad1.dpad_down);
            boolean raiseBallArm = (gamepad1.dpad_up);

            //gamepad2
            double raiseLowerLift = gamepad2.left_stick_y;
            double leftClamp = gamepad2.left_trigger;
            double rightClamp = gamepad2.right_trigger;
            boolean leftRelease = gamepad2.left_bumper;
            boolean rightRelease = gamepad2.right_bumper;
            boolean extendRelicArm = gamepad2.dpad_right;
            boolean retractRelicArm = gamepad2.dpad_left;
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

            if (slowMode) {
                speedFactor = 0.8; //was 0.75; //was 0.9 Bill wants to change to 0.8 on 2/10 (not tested yet)
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
                    if (TELE) telemetry.addData("Gripper Lift", "Raising");
                } else if (raiseLowerLift > 0.09/*0.2*/) {
                    gripperLiftMotor.setPower(raiseLowerLift/*1.0*/); //if trigger is negative, lower lift
                    if (TELE) telemetry.addData("Gripper Lift", "Lowering");
                } else {
                    gripperLiftMotor.setPower(0.0); //this should do motor braking
                    if (TELE) telemetry.addData("Gripper Lift", "Stopped");
                }
                if (TELE) telemetry.addData("Gripper Position",gripperLiftMotor.getCurrentPosition());

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
                    if (vacuumRunning) {
                        if (TELE) telemetry.addData("Vacuum1", "On");
                        vacuumMotor.setPower(1.0);
                    } else {
                        if (TELE) telemetry.addData("Vacuum1", "Off");
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
                    if (vacuumRunning2) {
                        if (TELE) telemetry.addData("Vacuum2", "On");
                        vacuumMotor2.setPower(1.0);
                    } else {
                        if (TELE) telemetry.addData("Vacuum2", "Off");
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
                        relicLiftServo.setPosition(relicLiftPosition);
                    } else if (lowerRelic) { //step it down
                        relicLiftPosition -= relicLiftSpeed * (currentLoopTime - lastLoopTime);
                        if (relicLiftPosition < 0.0) relicLiftPosition = 0.0;
                        relicLiftServo.setPosition(relicLiftPosition);
                    }
                    if (TELE) telemetry.addData("Relic Lift", relicLiftPosition);
                }

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
                    if (TELE) telemetry.addData("Arm Lift", armLiftPosition);

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
                    if (TELE) telemetry.addData("Gripper Twist", gripperTwistPosition);

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
                    if (TELE) telemetry.addData("Relic Claw", relicClawPosition);

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
                    if (TELE) telemetry.addData("Block Tilt", blockTiltPosition);

                }

                if (vacuumReleaseServo != null) {
                    if (vacuumRelease1>0.09) {
                        vacuumReleasePosition=0.0; //flipper both of these on 2/10 (again)
                    } else {
                        vacuumReleasePosition=1.0;
                    }
                    vacuumReleaseServo.setPosition(vacuumReleasePosition);
                    if (TELE) telemetry.addData("Vacuum Release1", vacuumReleasePosition);
                }

                if (vacuumReleaseServo2 != null) {
                    if (vacuumRelease2>0.09) {
                        vacuumReleasePosition2=0.0;
                    } else {
                        vacuumReleasePosition2=1.0;
                    }
                    vacuumReleaseServo2.setPosition(vacuumReleasePosition2);
                    if (TELE) telemetry.addData("Vacuum Release2", vacuumReleasePosition2);
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
                    if (TELE) telemetry.addData("Ball Arm", ballArmPosition);
                }

                if (ballColorSensor != null)
                    if (TELE) telemetry.addData("BallColor", "R=%d G=%d B=%d A=%d", ballColorSensor.red(), ballColorSensor.green(), ballColorSensor.blue(), ballColorSensor.alpha());

                if (bottomColorSensor != null)
                    if (TELE) telemetry.addData("BottomColor", "R=%d G=%d B=%d A=%d", bottomColorSensor.red(), bottomColorSensor.green(), bottomColorSensor.blue(), bottomColorSensor.alpha());

                /*
                int lfpos = lf.getCurrentPosition(); //show positions to help with auto mode
                int rfpos = rf.getCurrentPosition();
                int lbpos = lb.getCurrentPosition();
                int rbpos = rb.getCurrentPosition();

                telemetry.addData("Positions", "lf=%d rf=%d lb=%d rb=%d", lfpos, rfpos, lbpos, rbpos);
                */
                if (gyro != null)

                {
                    int heading = gyro.getHeading();
                    if (TELE) telemetry.addData("Heading", "%3d degrees", heading);
                }
                // Show the elapsed game time
                if (TELE) telemetry.addData("Status", "Run Time: " + runtime.toString());
                if (TELE) telemetry.update();
            }
        }
    }
}

