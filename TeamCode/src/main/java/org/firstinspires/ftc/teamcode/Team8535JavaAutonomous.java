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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

//@Autonomous(name="JavaAutoR", group="Autonomous")
//Now not using the gripper so using block tilt to insert the cube
//Still need to decide what our autonomous run is going to be like


public class Team8535JavaAutonomous extends LinearOpMode {

    public static final int ALLIANCE_RED=1;
    public static final int ALLIANCE_BLUE=2;

    public static final int SIDE_LEFT=1;
    public static final int SIDE_RIGHT=2;

    public static final int BALL_RED=1;
    public static final int BALL_BLUE=2;

    public static double BALL_ARM_UP=0; //fill these in after testing on prod bot
    public static double BALL_ARM_DOWN=0.7; //fill these in after testing on prod bot //was 0.5 in last event

    //VuMarks
    VuforiaLocalizer vuforia;

    private int alliance;
    private int side;

    private boolean prodbot = false;

    //Speed Factor for Fast/Slow Mode
    private double speedFactor = 1.0; //default full speed

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private double vacuumTime = 0.0;
    private double vacuumTime2 = 0.0; //added for second vacuum pump
    private double lastLoopTime=0.0;
    private double currentLoopTime=0.0;
    private double lastHold=0.0;

    //Drive Motors
    private DcMotor lf=null;
    private DcMotor rf=null;
    private DcMotor lb=null;
    private DcMotor rb=null;

    //Front Gripper
    private DcMotor gripperLiftMotor = null;
    private Servo gripperLeftServo = null;
    private Servo gripperRightServo = null;

    private double gripperLeftPosition = 0.0; //initial position of left gripper (tune this)
    private double gripperRightPosition = 1.0; //initial position of right gripper (tune this)
    private double gripperLeftClosingSpeed = 4.0; //range per second
    private double gripperRightClosingSpeed = 4.0; //range per second
    private double gripperLeftOpeningSpeed = 5.0; //range per second
    private double gripperRightOpeningSpeed = 5.0; //range per second

    //Ball Arm
    private Servo ballArmServo; //we'll need a servo to raise/lower the ball arm
    private double ballArmPosition = 0.0;//initial position of ball arm servo (tune this)
    private double ballArmSpeed = 0.5; //range per second

    //Block Tilt
    private Servo blockTiltServo = null;
    private double blockTiltPosition = 1.0;
    private double blockDumpPosition = 0.6;
    private double blockTiltSpeed = 0.7;

    //Vacuum
    private DcMotor vacuumMotor = null;
    private DcMotor vacuumMotor2 = null; //added
    private Servo vacuumReleaseServo = null;
    private Servo vacuumReleaseServo2 = null; //added

    private double vacuumReleasePosition = 0.5;
    private double vacuumReleasePosition2 = 0.5;

    private double vacuumReleaseSpeed = 1.0;

    private boolean vacuumRunning = false;
    private boolean vacuumRunning2 = false;

    //Base
    private ColorSensor bottomColorSensor = null;

    //Ball Color Sensor
    ColorSensor ballColorSensor; //we'll need a color sensor to detect ball color
    private int ballColor=0;

    //Gyro Sensor
    ModernRoboticsI2cGyro gyro; //a gyro would be really useful

    //private DcMotor vacuum=null;
    //private DcMotor vacuumRelease=null; //this will be eliminated or change to standard servo

    private static boolean SHOW_CAMERA=true; //whether to show the camera on the phone screen
    private static boolean JOYSTICK_SCALING=true; //whether to scale joystick values by cubing value (more precision for small movements)

    private static final int STATE_START=0; //at start of the run
    private static final int STATE_LOOKING=1; //actively looking for the vumark
    private static final int STATE_MOVE_ARM_DOWN=2; //moving arm down
    private static final int STATE_SENSE_BALL_COLOR=3; //sensing the ball's color
    private static final int STATE_ROTATE_BALL_OFF=4; //rotating the arm to knock the ball off
    private static final int STATE_MOVE_ARM_UP=5; // moving arm up
    private static final int STATE_ROTATE_BACK=6; //rotating the arm back to the way it was
    private static final int STATE_MOVING=7; //moving to cryptobox
    private static final int STATE_ROTATE_TO_CRYPTOBOX=9; //rotate the robot to face the cryptobox
    private static final int STATE_PUSH_BLOCK_TO_CRYPTOBOX =10;
    private static final int STATE_RELEASING_BLOCK=11;

    private static final int STATE_LIFTING_GRIPPER=12;
    private static final int STATE_OPENING_GRIPPER=13;
    private static final int STATE_LOWERING_GRIPPER=14;
    private static final int STATE_CLOSING_GRIPPER=15;
    private static final int STATE_BACKING_UP=16;

    private static final int STATE_START_LIFT=17;

    private static final int STATE_MOVE_CLOSE=18;

    private static final int STATE_DUMPING_BLOCK = 19;

    private static final int STATE_MOVE_BACK = 20;

    private static final int STATE_MOVE_IN = 21;

    private static final int STATE_MOVE_OUT = 22;

    private static final int STATE_DONE=23;

    private static final int STATE_EXTRA_MOVE = 24;

    private static final int STATE_EXTRA_ROTATE = 25;

    private static String[] stateNames={"Starting","Looking","Moving Arm Down","Sensing Ball Color",
            "Rotating to Knock Ball Off","Moving Arm Up","Rotating Back","Moving to CryptoBox",
            "At CryptoBox","Rotating To CryptoBox","Pushing Block To CryptoBox","Releasing Block",
            "Lifting Gripper","Opening Gripper","Lowering Gripper","Closing Gripper","Backing Up","Starting Lift","Moving Close to Cryptobox","Dumping Block","Move Back","Move In", "Move Out","Done", "Extra Move", "Extra Rotate"
    }; //state names for telemetry

    private int state; //the current state our robot is in
    RelicRecoveryVuMark vuMark; //currently detect vuMark

    //persistent telemetry items
    private String ballSeen="";
    private String posterSeen="";

    public Team8535JavaAutonomous(int alliance,int side) {
        this.alliance=alliance;
        this.side=side;
    }

    private static Map<RelicRecoveryVuMark,Integer> distMap=new HashMap<RelicRecoveryVuMark,Integer>();

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

    private ColorSensor getColorSensor(String sensorName) { //these could be made generic using type notation
        try {
            return(hardwareMap.get(ColorSensor.class,sensorName));
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

    private ModernRoboticsI2cGyro getGyro(String gyroName) { //these could be made generic using type notation
        try {
            return(hardwareMap.get(ModernRoboticsI2cGyro.class,gyroName));
        } catch (Exception e) {
            return(null);
        }
    }

    private boolean needsExtra() {
        return((alliance == ALLIANCE_BLUE && side == SIDE_RIGHT) || (alliance == ALLIANCE_RED && side == SIDE_LEFT));
    }

    /**
     * For debugging pause the state machine
     * @param time current elapsed time counter
     */
    private void holdup(ElapsedTime time) {
        telemetry.addData("Next State",stateNames[state]); //show the next planned state
        telemetry.update(); //update telemetry before the pause
        //while(!gamepad1.x && (time.time()-lastHold)>1.0) {}; //wait for X to be hit to proceed
        //lastHold=time.time(); //debounce X
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
        /*
        int current=gyro.getHeading();
        int target=current+degrees;
        telemetry.addData("Gyro Turn","At %d Target %d",current,target);
        telemetry.update();
        */
        double startTime=runtime.time();
        if (degrees>0) {
            mecanumMoveNoScale(0,0,-0.3);
            while(true/*gyro.getHeading()<target*/) {
                //telemetry.addData("Turning","At %d",gyro.getHeading());
                //telemetry.update();
                if ((runtime.time()-startTime)>maxTime) break;
            }
        } else {
            mecanumMoveNoScale(0,0,0.3);
            while(true/*gyro.getHeading()>target*/) {
                //telemetry.addData("Turning","At %d",gyro.getHeading());
                //telemetry.update();
                if ((runtime.time()-startTime)>maxTime) break;
            }
        }
        mecanumMoveNoScale(0,0,0);
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
        if (getDevice("drivebot") != null) {
            prodbot = false;
            telemetry.addData("Bot", "DriveBot");
        } else {
            prodbot = true;
            telemetry.addData("Bot", "ProdBot");
        }
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        lf  = hardwareMap.get(DcMotor.class, "lf");
        rf  = hardwareMap.get(DcMotor.class, "rf");
        lb  = hardwareMap.get(DcMotor.class, "lb");
        rb  = hardwareMap.get(DcMotor.class, "rb");

        //Gripper
        /*
        gripperLiftMotor = getMotor("gripper_lift");
        if (gripperLiftMotor!=null) gripperLiftMotor.setPower(0.0);
        gripperLeftServo = getServo("gripper_left");
        //if (gripperLeftServo != null) gripperLeftServo.setPosition(gripperLeftPosition);
        gripperRightServo = getServo("gripper_right");
        if (gripperRightServo != null) gripperRightServo.setPosition(gripperRightPosition);
        */

        //Ball Arm
        ballArmServo = getServo("ball_arm");
        if (ballArmServo !=null) ballArmServo.setPosition(ballArmPosition);
        ballColorSensor = getColorSensor("ball_color");

        //Block Tilt
        blockTiltServo = getServo("block_tilt");
        if (blockTiltServo !=null) blockTiltServo.setPosition(blockTiltPosition);
        /*
        bottomColorSensor = getColorSensor("bottom_color");
        if (bottomColorSensor!=null) bottomColorSensor.setI2cAddress(I2cAddr.create8bit(0x48)); //we believe these are 7bit addresses
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
        */

        //vacuumRelease  = hardwareMap.get(DcMotor.class, "release");

        lf.setDirection(DcMotor.Direction.REVERSE); //was REVERSE
        rf.setDirection(DcMotor.Direction.FORWARD); //was REVERSE
        lb.setDirection(DcMotor.Direction.REVERSE); //was FORWARD
        rb.setDirection(DcMotor.Direction.FORWARD); //was FORWARD

        if (gripperLeftServo!=null) gripperLeftServo.setDirection(Servo.Direction.REVERSE); //changed to reverse to flip left gripper servo direction
        if (gripperRightServo!=null) gripperRightServo.setDirection(Servo.Direction.FORWARD);
        if (ballArmServo!=null) ballArmServo.setDirection(Servo.Direction.FORWARD);

        if (prodbot) {
            lf.setDirection(DcMotor.Direction.REVERSE); //was REVERSE
            rf.setDirection(DcMotor.Direction.FORWARD); //was REVERSE
            lb.setDirection(DcMotor.Direction.REVERSE); //was FORWARD
            rb.setDirection(DcMotor.Direction.FORWARD); //was FORWARD

            if (gripperLiftMotor!=null) gripperLiftMotor.setDirection(DcMotor.Direction.FORWARD);

        } else {
            lf.setDirection(DcMotor.Direction.REVERSE); //was REVERSE
            rf.setDirection(DcMotor.Direction.FORWARD); //was REVERSE
            lb.setDirection(DcMotor.Direction.REVERSE); //was FORWARD
            rb.setDirection(DcMotor.Direction.FORWARD); //was FORWARD

            if (gripperLiftMotor!=null) gripperLiftMotor.setDirection(DcMotor.Direction.FORWARD);
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        relicTrackables.activate();

        double time = 0; //move timer

        distMap.put(RelicRecoveryVuMark.LEFT,1350); //in milliseconds of movement
        distMap.put(RelicRecoveryVuMark.CENTER,1600);
        distMap.put(RelicRecoveryVuMark.RIGHT,1850);

        state=STATE_START; //in the start state

        while (opModeIsActive()) {

            telemetry.addData("State",stateNames[state]); //echo the state we're in
            telemetry.addData("Poster Seen",posterSeen);
            telemetry.addData("Ball Seen",ballSeen);

            //use a switch statement to take action based on the state we're in
            switch(state) {
                case STATE_START:
                    time = runtime.milliseconds();
                    state = STATE_LOOKING; //start looking for the VuMark
                    break;
/*
                case STATE_START_LIFT:
                    time = runtime.milliseconds();
                    gripperLiftMotor.setPower(-1.0);
                    state = STATE_LIFTING_GRIPPER;
                    break;

                case STATE_LIFTING_GRIPPER:
                    telemetry.addData("Lift Gripper", gripperLiftMotor.getCurrentPosition());
                    if ((runtime.milliseconds() - time) > 500) {
                        gripperLiftMotor.setPower(0.0);
                        state = STATE_OPENING_GRIPPER;
                        holdup(runtime);
                    }
                    break;

                case STATE_OPENING_GRIPPER:
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                    }
                    ;
                    gripperLeftServo.setPosition(0.3);
                    gripperRightServo.setPosition(0.1);
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                    }
                    ;
                    state = STATE_LOWERING_GRIPPER;
                    holdup(runtime);
                    time = runtime.milliseconds();
                    gripperLiftMotor.setPower(1.0);
                    break;

                case STATE_LOWERING_GRIPPER:
                    telemetry.addData("Lift Gripper", gripperLiftMotor.getCurrentPosition());
                    if ((runtime.milliseconds() - time) > 500) {
                        gripperLiftMotor.setPower(0.0);
                        state = STATE_CLOSING_GRIPPER;
                        holdup(runtime);
                    }
                    break;

                case STATE_CLOSING_GRIPPER:
                    gripperLeftServo.setPosition(1.0);
                    gripperRightServo.setPosition(1.0);
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                    }
                    ;
                    state = STATE_MOVE_ARM_DOWN;
                    holdup(runtime);
                    break;
*/
                case STATE_LOOKING:
                    vuMark = RelicRecoveryVuMark.from(relicTemplate);
                    if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                        telemetry.addData("VuMark", "%s visible", vuMark);
                        posterSeen=vuMark.toString();
                        state = STATE_MOVE_ARM_DOWN; //STATE_MOVE_ARM_DOWN; //STATE_START_LIFT //if picking up block
                        holdup(runtime);
                    } else if ((runtime.milliseconds() - time) > 5000) {
                        telemetry.addData("VuMark", "Assuming Center");
                        posterSeen="Unknown";
                        vuMark = RelicRecoveryVuMark.CENTER; //assume center after 5 seconds
                        state = STATE_MOVE_ARM_DOWN;
                        holdup(runtime);
                    }
                    break;

                case STATE_MOVE_ARM_DOWN:
                    ballArmServo.setPosition(BALL_ARM_DOWN);
                    try {
                        Thread.sleep(2000);
                    } catch (InterruptedException e) {
                    }
                    ;
                    state = STATE_SENSE_BALL_COLOR;
                    holdup(runtime);
                    //ballArmServo.setPosition(BALL_ARM_DOWN);
                    //need to wait a bit -- use technique we used with move
                    //should move servo arm to down position
                    break;

                case STATE_SENSE_BALL_COLOR:
                    telemetry.addData("BallColor", "R=%d G=%d B=%d A=%d", ballColorSensor.red(), ballColorSensor.green(), ballColorSensor.blue(), ballColorSensor.alpha());
                    if ((ballColorSensor.red() != 0 && ballColorSensor.red() != 255)
                            || ballColorSensor.blue() != 0 && ballColorSensor.blue() != 255) {
                        if (ballColorSensor.red() > ballColorSensor.blue()) {
                            ballColor = BALL_RED;
                            telemetry.addData("Ball Color", "Red");
                            ballSeen=String.format("Red red=%d blue=%d",ballColorSensor.red(),ballColorSensor.blue());
                            telemetry.update();
                        } else if (ballColorSensor.blue() > ballColorSensor.red()) {
                            ballColor = BALL_BLUE;
                            telemetry.addData("Ball Color", "Blue");
                            ballSeen=String.format("Blue red=%d blue=%d",ballColorSensor.red(),ballColorSensor.blue());
                            telemetry.update();
                        } else {
                            telemetry.addData("Ball Color", "Unknown");
                            ballSeen="Unknown";
                            telemetry.update();
                        }
                        time = runtime.milliseconds();

                        state = STATE_ROTATE_BALL_OFF;
                        holdup(runtime);
                    } else {
                        state = STATE_MOVE_ARM_UP;
                        holdup(runtime);
                    }
                    //blue alliance: (facing toward shelves) if blue then rotate 20, if red then rotate -20
                    //red alliance: (facing away from shelves) if blue then rotate 20, if red then rotate -20
                    //should read the color sensor and record color (using comparison of blue to red)
                    break;

                case STATE_ROTATE_BALL_OFF:
                    if (alliance == ALLIANCE_RED) { //forward to cryptobox
                        if (ballColor == BALL_BLUE) { //blue in front of us
                            mecanumMoveNoScale(0, -0.5, 0); //move forward
                        } else { //blue in back of us
                            mecanumMoveNoScale(0, 0.5, 0); //move backward
                        }
                    } else if (alliance == ALLIANCE_BLUE) { //backward to cryptobox
                        if (ballColor == BALL_BLUE) { //red in back of us
                            mecanumMoveNoScale(0, 0.5, 0); //move backward
                        } else { //red in front of us
                            mecanumMoveNoScale(0, -0.5, 0); //move forward
                        }
                    }
                    //telemetry.addData("Knocking Ball Off");
                    if ((runtime.milliseconds() - time) > 300) { //was 300
                        mecanumMoveNoScale(0.0,0.0,0.0);
                        state = STATE_MOVE_ARM_UP; //after a second were at cryptobox?
                        holdup(runtime);
                        try { Thread.sleep(500); } catch (InterruptedException e) {};
                    }
                    break;


                case STATE_MOVE_ARM_UP:
                    ballArmServo.setPosition(BALL_ARM_UP);
                    try {Thread.sleep(1000);} catch(InterruptedException e) {};
                    time = runtime.milliseconds();
                    state = STATE_MOVING;
                    holdup(runtime);
                    //ballArmServo.setPosition(BALL_ARM_UP);
                    //need to wait a bit -- use technique we used with move
                    //should move servo arm to up position
                    break;
/*
                case STATE_ROTATE_BACK:
                    //should rotate bot opposite of previous rotate step
                    if (ballColor == BALL_BLUE) {
                        gyroTurn(5, runtime, 1.0);
                    } else if (ballColor == BALL_RED) {
                        gyroTurn(-5, runtime, 1.0);
                    }
                    try { Thread.sleep(1000); } catch (InterruptedException e) {};
                    state=STATE_MOVING;
                    holdup(runtime);
                    time = runtime.milliseconds();
                    break;
*/
                case STATE_MOVING:
                    //should move by time, encoders, or inertial
                    if (alliance == ALLIANCE_RED) {
                        mecanumMoveNoScale(0, -0.5, 0); //move forward
                    } else {
                        mecanumMoveNoScale(0, 0.5, 0); //move backward
                    }
                    int moveTime = 2400;
                    if (needsExtra()) {
                        moveTime = 1800;
                    }
                    //later remember to compensate for ball knocking move
                    telemetry.addData("Moving", "%s units", distMap.get(vuMark));
                    if ((runtime.milliseconds() - time) > moveTime) { //1600 goes to 2400
                        mecanumMoveNoScale(0.0,0.0,0.0);
                        if (needsExtra()) {
                            state = STATE_EXTRA_MOVE;
                        }else {
                            state = STATE_MOVE_CLOSE; //after a second were at cryptobox?
                        }
                        holdup(runtime);
                        try { Thread.sleep(500); } catch (InterruptedException e) {};
                        time = runtime.milliseconds();
                    }

                    break;

                case STATE_EXTRA_MOVE:
                    mecanumMoveNoScale(-1, 0, 0); //strafe left
                    if ((runtime.milliseconds() - time) > 950) { //was 850
                        mecanumMoveNoScale(0.0,0.0,0.0);
                        state = STATE_EXTRA_ROTATE; //
                        holdup(runtime);
                        try { Thread.sleep(500); } catch (InterruptedException e) {};
                        time = runtime.milliseconds();
                    }
                    break;

                case STATE_EXTRA_ROTATE:
                    mecanumMoveNoScale(0, 0, 1); //rotate counter-clockwise
                    if ((runtime.milliseconds() - time) > 550) {
                        mecanumMoveNoScale(0.0,0.0,0.0);
                        state = STATE_MOVE_CLOSE; //
                        holdup(runtime);
                        try { Thread.sleep(500); } catch (InterruptedException e) {};
                        time = runtime.milliseconds();
                    }
                    break;

                case STATE_MOVE_CLOSE:
                    mecanumMoveNoScale(1, 0, 0); //move right
                    telemetry.addData("Moving Closer to Cryptobox", "");
                    if ((runtime.milliseconds() - time) > 500) { //1600 goes to 2400
                        mecanumMoveNoScale(0.0,0.0,0.0);
                        state = STATE_DUMPING_BLOCK; //after a second were at cryptobox?
                        holdup(runtime);
                        try { Thread.sleep(500); } catch (InterruptedException e) {};
                    }
                    break;

                case STATE_DUMPING_BLOCK:
                    if (blockTiltServo !=null) blockTiltServo.setPosition(blockDumpPosition);
                    state = STATE_MOVE_BACK; //after a second were at cryptobox?
                    holdup(runtime);
                    try { Thread.sleep(500); } catch (InterruptedException e) {};
                    time = runtime.milliseconds();
                    break;

                case STATE_MOVE_BACK:
                    mecanumMoveNoScale(-1, 0, 0); //move left
                    telemetry.addData("Moving Closer to Cryptobox", "");
                    if ((runtime.milliseconds() - time) > 500) { //1600 goes to 2400
                        mecanumMoveNoScale(0.0,0.0,0.0);
                        state = STATE_MOVE_IN; //after a second were at cryptobox?
                        holdup(runtime);
                        try { Thread.sleep(500); } catch (InterruptedException e) {};
                        time = runtime.milliseconds();
                    }
                    break;

                case STATE_MOVE_IN:
                    mecanumMoveNoScale(1, 0, 0); //move right
                    telemetry.addData("Moving Closer to Cryptobox", "");
                    if ((runtime.milliseconds() - time) > 500) { //1600 goes to 2400
                        mecanumMoveNoScale(0.0,0.0,0.0);
                        state = STATE_MOVE_OUT; //after a second were at cryptobox?
                        holdup(runtime);
                        try { Thread.sleep(500); } catch (InterruptedException e) {};
                        time = runtime.milliseconds();

                    }
                    break;

                case STATE_MOVE_OUT:
                    mecanumMoveNoScale(-0.75, 0, 0); //move more slowly left
                    telemetry.addData("Moving Closer to Cryptobox", "");
                    if ((runtime.milliseconds() - time) > 500) { //1600 goes to 2400
                        mecanumMoveNoScale(0.0,0.0,0.0);
                        state = STATE_DONE; //after a second were at cryptobox?
                        holdup(runtime);
                        try { Thread.sleep(500); } catch (InterruptedException e) {};
                    }
                    break;
/*
                case STATE_ROTATE_TO_CRYPTOBOX:
                    gyroTurn(-90,runtime,4.0);
                    state= STATE_PUSH_BLOCK_TO_CRYPTOBOX;
                    holdup(runtime);
                    time = runtime.milliseconds();
                    mecanumMoveNoScale(0.0,-0.5,0.0);
                    break;

                case STATE_PUSH_BLOCK_TO_CRYPTOBOX:
                    if ((runtime.milliseconds() - time) > 750) {
                        mecanumMoveNoScale(0.0,0.0,0.0);
                        state = STATE_RELEASING_BLOCK; //after a second were at cryptobox?
                        holdup(runtime);
                    }
                    break;

                case STATE_RELEASING_BLOCK:
                    gripperLeftServo.setPosition(0.0);
                    gripperRightServo.setPosition(0.0);
                    try {Thread.sleep(500);} catch(InterruptedException e) {};
                    state = STATE_BACKING_UP;
                    holdup(runtime);
                    time=runtime.milliseconds();
                    mecanumMoveNoScale(0.0,0.5,0.0);
                    break;

                case STATE_BACKING_UP:
                    if ((runtime.milliseconds() - time) > 750) {
                        mecanumMoveNoScale(0.0,0.0,0.0);
                        state = STATE_DONE;
                        requestOpModeStop();
                    }
                    break;
                 */
            }

            // Show the elapsed game time
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
