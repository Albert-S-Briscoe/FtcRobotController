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
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Test Program", group="Linear Opmode")

public class TestProgram extends LinearOpMode {
    
    boolean[] button = {false,false,false,false};
    boolean wheelsOn = false;
    private ElapsedTime runtime = new ElapsedTime();
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        RobotHardware H = new RobotHardware();
        H.init(hardwareMap);
        MecanumWheelDriver driver = new MecanumWheelDriver(H, this);
        
        waitForStart();
        runtime.reset();
        H.launchServo.setPosition(0.02);
        
        while (opModeIsActive()) {
            
            //H.launcherServo.setPosition((gamepad1.left_stick_x+1)/2);
    
            if (gamepad1.a) {
        
                if (!button[0]) {
                    driver.setMoveInches(180, 50, 1, 180);
                    driver.MoveInches();
                    //H.launchServo.setPosition(0.17);
                    //sleep(100);
                    //H.launchServo.setPosition(0.02);
                    button[0] = true;
            
                }
        
            } else {
    
                button[0] = false;
        
            }
    
            if (gamepad1.b) {
        
                if (!button[1]) {
            
                    wheelsOn = !wheelsOn;
                    button[1] = true;
            
                }
        
            } else {
        
                button[1] = false;
        
            }
    
            if (gamepad1.x) {
        
                if (!button[2]) {
    
                    H.launchServo.setPosition(0.17);
                    sleep(100);
                    H.launchServo.setPosition(0.02);
                    sleep(120);
                    H.launchServo.setPosition(0.17);
                    sleep(100);
                    H.launchServo.setPosition(0.02);
                    sleep(120);
                    H.launchServo.setPosition(0.17);
                    sleep(100);
                    H.launchServo.setPosition(0.02);
                    sleep(120);
                    button[2] = true;
            
                }
        
            } else {
        
                button[2] = false;
        
            }
    
            if (gamepad1.y) {
        
                if (!button[3]) {
                    driver.setrotate(90, 1, true);
                    driver.rotate();
                    //H.launchServo.setPosition(0.17);
                    //sleep(100);
                    //H.launchServo.setPosition(0.02);
                    button[3] = true;
            
                }
        
            } else {
        
                button[3] = false;
        
            }
            
            if (wheelsOn) {
                //H.driveMotor[0].setPower(1);
                H.driveMotor[1].setPower(1);
            } else {
                //H.driveMotor[0].setPower(0);
                H.driveMotor[1].setPower(0);
            }
            
            telemetry.addData("Time: " , runtime.seconds());
            telemetry.addData("Pos: " , (gamepad1.left_stick_x+1)/2);
            telemetry.addData("deg:", H.getheading());
            telemetry.update();
        }
        
    }
}