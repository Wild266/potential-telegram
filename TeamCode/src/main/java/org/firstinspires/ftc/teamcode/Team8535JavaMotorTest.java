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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@TeleOp(name="JavaMotorTest", group="Linear Opmode")
public class Team8535JavaMotorTest extends LinearOpMode {

    private boolean prodbot = false;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;

    private I2cDevice getDevice(String deviceName) { //these could be made generic using type notation
        try {
            return (hardwareMap.get(I2cDevice.class, deviceName));
        } catch (Exception e) {
            return (null);
        }
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

        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");

        if (prodbot) {
            lf.setDirection(DcMotor.Direction.REVERSE);  //reverse
            rf.setDirection(DcMotor.Direction.REVERSE);  //forward
            lb.setDirection(DcMotor.Direction.REVERSE);  //reverse
            rb.setDirection(DcMotor.Direction.FORWARD);  //forward
        } else {
            lf.setDirection(DcMotor.Direction.REVERSE);  //reverse
            rf.setDirection(DcMotor.Direction.FORWARD);  //forward
            lb.setDirection(DcMotor.Direction.REVERSE);  //reverse
            rb.setDirection(DcMotor.Direction.FORWARD);  //forward
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoders
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {

            if (gamepad1.x) { //stop and look for vumarks if X key is down
                lf.setPower(gamepad1.left_stick_x);
            } else {
                lf.setPower(0.0);
            }

            if (gamepad1.y) { //stop and look for vumarks if Y key is down
                rf.setPower(gamepad1.left_stick_x);
            } else {
                rf.setPower(0.0);
            }

            if (gamepad1.b) { //stop and look for vumarks if B key is down
                rb.setPower(gamepad1.left_stick_x);
            } else {
                rb.setPower(0.0);
            }

            if (gamepad1.a) { //stop and look for vumarks if A key is down
                lb.setPower(gamepad1.left_stick_x);
            } else {
                lb.setPower(0.0);
            }
        }

    }

}
