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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

class Device {
    String name;
    Object device;
    String type;

    public Device(String name,Object device,String type) {
        this.name=name;
        this.device=device;
        this.type=type;
    }
}

@TeleOp(name="JavaUniversalTester", group="Linear Opmode")
public class Team8535UniversalTester extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private double lastLoopTime;
    private double currentLoopTime;

    private DcMotor getMotor(String motorName) { //these could be made generic using type notation
        try {
            return (hardwareMap.get(DcMotor.class, motorName));
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

    private String[] motorNames={"lf","rf","rb","lb","vacuum","vacuum2","arm_extend","gripper_lift"};
    private String[] servoNames={"gripper_twist", "block_tilt","relic_claw","relic_lift","ball_arm","arm_lift","vacuum_release","vacuum_release2"};

    private ArrayList<Device> devices=new ArrayList<Device>();

    private int current=0;
    private Device currentDevice=null;

    private double lastButtonTime=0;

    private double servoPosition=0.5;
    private double servoSpeed=1.0;

    private void buildDeviceList() {
        for(String name:motorNames) {
            Device dev=new Device(name,getMotor(name),"motor");
            if (dev.device!=null) {
                devices.add(dev);
            }
        }
        for(String name:servoNames) {
            Device dev=new Device(name,getServo(name),"servo");
            if (dev.device!=null) {
                devices.add(dev);
            }
        }
    }

    private Device getCurrentDevice() {
        if (devices.size()==0) buildDeviceList();
        currentDevice=devices.get(current);
        return(currentDevice);
    }

    private Device getNextDevice() {
        currentDevice=getCurrentDevice();
        current++;
        if (current >= devices.size()) current = 0;
        currentDevice = getCurrentDevice();
        return (currentDevice);
    }

    @Override
    public void runOpMode() {
        getCurrentDevice();
        telemetry.addData("Device",currentDevice.name);
        telemetry.addData("Device Type",currentDevice.type);
        telemetry.update();
        waitForStart();
        runtime.reset();
        currentLoopTime=runtime.time();
        while (opModeIsActive()) {
            lastLoopTime=currentLoopTime; //the loop timing difference gives our actively stepping servos the time to use with their speed multipliers
            currentLoopTime=runtime.time();
            if (gamepad1.x && ((runtime.time()-lastButtonTime)>0.3)) {
                getNextDevice();
                lastButtonTime=runtime.time();
                if (currentDevice.type=="servo") {
                    servoPosition = 0.5;
                    ((Servo)currentDevice.device).setPosition(servoPosition);
                }
            }
            telemetry.addData("Device",currentDevice.name);
            telemetry.addData("Device Type",currentDevice.type);

            if (currentDevice.type=="motor") {
                if (gamepad1.left_stick_x<-0.3) {
                    ((DcMotor)currentDevice.device).setPower(-1.0);
                    telemetry.addData("Motor Direction","Negative");
                } else if (gamepad1.left_stick_x>0.3) {
                    ((DcMotor)currentDevice.device).setPower(1.0);
                    telemetry.addData("Motor Direction","Positive");
                } else {
                    ((DcMotor)currentDevice.device).setPower(0.0);
                    telemetry.addData("Motor Direction","Braking");
                }
            }
            if (currentDevice.type=="servo") {
                if (gamepad1.left_stick_x>0.3) { //step up
                    servoPosition+=servoSpeed*(currentLoopTime-lastLoopTime);
                    if (servoPosition>1.0) servoPosition=1.0;
                } else if (gamepad1.left_stick_x<-0.3) { //step down
                    servoPosition-=servoSpeed*(currentLoopTime-lastLoopTime);
                    if (servoPosition<0.0) servoPosition=0.0;
                }
                ((Servo)currentDevice.device).setPosition(servoPosition);
                telemetry.addData("Servo Position",servoPosition);
            }
            telemetry.update();
        }

    }

}
