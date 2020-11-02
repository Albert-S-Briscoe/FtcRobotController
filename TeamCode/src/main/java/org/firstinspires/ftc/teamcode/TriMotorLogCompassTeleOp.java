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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@Disabled
@TeleOp(name="TeleOp Tri", group="Linear Opmode")
public class TriMotorLogCompassTeleOp extends LinearOpMode {
    
    Orientation angles;
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        ElapsedTime      runtime     = new ElapsedTime();
        DcMotor          motorOne    = hardwareMap.get(DcMotor.class, "motor 1");
        DcMotor          motorTwo    = hardwareMap.get(DcMotor.class, "motor 2");
        DcMotor          motorThree  = hardwareMap.get(DcMotor.class, "motor 3");
        BNO055IMU        imu         = hardwareMap.get(BNO055IMU.class, "imu");
        
        BNO055IMU.Parameters parameters  = new BNO055IMU.Parameters();
        parameters.angleUnit             = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile   = "BNO055IMUCalibration.json";
        
        imu.initialize(parameters);
        
        motorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorThree.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        motorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorThree.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        double onePower;
        double twoPower;
        double threePower;
        double one;
        double two;
        double three;
        double multiplier;
        
        final double logCurve = 20;
        double y;
        double x;
        double Rotate;
        double Radius;
        double stickTotal;
        double Angle;
        double agl_frwd = 0;
        
        boolean button = false;
        boolean toggle = false;
        
        waitForStart();
        runtime.reset();
        
        while (opModeIsActive()) {
            
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            if (Math.abs(gamepad1.right_stick_x) > 0.05) {
                if (gamepad1.right_stick_x > 0) {
                    Rotate = -((Math.log10((-Range.clip(gamepad1.right_stick_x, -1, 1) + 1) * logCurve + 1)) / Math.log10(logCurve + 1)) + 1;
                } else {
                    Rotate = -(-((Math.log10((Range.clip(gamepad1.right_stick_x, -1, 1) + 1) * logCurve + 1)) / Math.log10(logCurve + 1)) + 1);
                }
            } else {
                Rotate = 0;
            }
            if (Math.hypot(x, y) > 0.05) {
                Radius = -((Math.log10((-Range.clip(Math.hypot(x, y), -1, 1) + 1) * logCurve + 1)) / Math.log10(logCurve + 1)) + 1;
            } else {
                Radius = 0;
            }
            stickTotal = Radius + Math.abs(Rotate);
            Angle = Math.atan2(y, x) + Math.toRadians(agl_frwd - AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) - 30);
            
            if (gamepad1.start && stickTotal < .1) {
                agl_frwd = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
            }
            
            one    = Math.sin(Angle);
            two    = Math.sin(Angle - 2.09);
            three  = Math.sin(Angle + 2.09);
            
            if (one > two && one > three) {
                multiplier = 1/one;
            } else if (two > one && two > three) {
                multiplier = 1/two;
            } else {
                multiplier = 1/three;
            }
            
            onePower    = one   * multiplier * Radius - Rotate;
            twoPower    = two   * multiplier * Radius - Rotate;
            threePower  = three * multiplier * Radius - Rotate;
            
            if (Math.abs(stickTotal) > 1) {
                onePower /= stickTotal;
                twoPower /= stickTotal;
                threePower /= stickTotal;
            }
    
            if (gamepad1.left_bumper) {
        
                if (!button) {
            
                    toggle = !toggle;
                    button = true;
            
                }
        
            } else {
        
                button = false;
        
            }
        
            if (toggle) {
    
                motorOne.setPower(onePower/2);
                motorTwo.setPower(twoPower/2);
                motorThree.setPower(threePower/2);
                
            } else {
    
                motorOne.setPower(onePower);
                motorTwo.setPower(twoPower);
                motorThree.setPower(threePower);
    
            }
            // Show run time, wheel power and sensor values.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("heading", "%.1f", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
            telemetry.addData("frontMotors", "one (%.2f), two (%.2f), three (%.2f)", onePower, twoPower, threePower);
            telemetry.update();
        }
    }
}
