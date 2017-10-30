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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.HashMap;
import java.util.Map;


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
 */

//@Autonomous(name="JavaAutoR", group="Autonomous")
public class Team8535JavaAutonomous extends LinearOpMode {

    public static final int ALLIANCE_RED=1;
    public static final int ALLIANCE_BLUE=2;

    public static final int SIDE_LEFT=1;
    public static final int SIDE_RIGHT=2;

    //VuMarks
    VuforiaLocalizer vuforia;

    private int alliance;
    private int side;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lf=null;
    private DcMotor rf=null;
    private DcMotor lb=null;
    private DcMotor rb=null;

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
    private static final int STATE_ROTATE_ARM_BACK=6; //rotating the arm back to the way it was
    private static final int STATE_MOVING=7; //moving to cryptobox
    private static final int STATE_AT_CRYPTOBOX=8; //at cryptobox
    private static final int STATE_ROTATE_TO_CRYPTOBOX=9; //rotate the robot to face the cryptobox

    private static String[] stateNames={"Starting","Looking","Moving","At CryptoBox"}; //state names for telemetry

    private int state; //the current state our robot is in
    RelicRecoveryVuMark vuMark; //currently detect vuMark

    public Team8535JavaAutonomous(int alliance,int side) {
        this.alliance=alliance;
        this.side=side;
    }

    private static Map<RelicRecoveryVuMark,Integer> distMap=new HashMap<RelicRecoveryVuMark,Integer>();

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
        double robotAngle = Math.atan2(-1*lsx,lsy) - Math.PI / 4;
        double rightX = -1 * rsx;
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

        //vacuum = hardwareMap.get(DcMotor.class, "vacuum");
        //vacuumRelease  = hardwareMap.get(DcMotor.class, "release");

        lf.setDirection(DcMotor.Direction.REVERSE); //was REVERSE
        rf.setDirection(DcMotor.Direction.REVERSE); //was REVERSE
        lb.setDirection(DcMotor.Direction.FORWARD); //was FORWARD
        rb.setDirection(DcMotor.Direction.FORWARD); //was FORWARD

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        relicTrackables.activate();

        double time = 0; //move timer

        distMap.put(RelicRecoveryVuMark.LEFT,2500);
        distMap.put(RelicRecoveryVuMark.CENTER,2000);
        distMap.put(RelicRecoveryVuMark.RIGHT,1500);

        state=STATE_START; //in the start state

        while (opModeIsActive()) {

            telemetry.addData("State",stateNames[state]); //echo the state we're in

            //use a switch statement to take action based on the state we're in
            switch(state) {
                case STATE_START:
                    state = STATE_LOOKING; //start looking for the VuMark
                    break;
                case STATE_LOOKING:
                    vuMark = RelicRecoveryVuMark.from(relicTemplate);
                    if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                        telemetry.addData("VuMark", "%s visible", vuMark);
                        time = runtime.milliseconds();
                        if (side == SIDE_LEFT) {
                            mecanumMove(0, 1, 0);

                        } else {
                            mecanumMove(0, -1, 0);
                        }
                    }
                        break;


                case STATE_MOVE_ARM_DOWN:

                    break;

                case STATE_SENSE_BALL_COLOR:

                    break;

                case STATE_ROTATE_BALL_OFF:

                    break;

                case STATE_MOVE_ARM_UP:

                    break;

                case STATE_ROTATE_ARM_BACK:

                    break;


            }

                case STATE_MOVING:
                    telemetry.addData("Moving", "%s units", distMap.get(vuMark));
                    if ((runtime.milliseconds()-time)>distMap.get(vuMark)) {
                        state=STATE_AT_CRYPTOBOX; //after a second were at cryptobox?
                    }

                    break;
                case STATE_AT_CRYPTOBOX:
                    mecanumMove(0,0,0); //stop
                    break;

                case STATE_ROTATE_TO_CRYPTOBOX:
                    break;
            }

            // Show the elapsed game time
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
