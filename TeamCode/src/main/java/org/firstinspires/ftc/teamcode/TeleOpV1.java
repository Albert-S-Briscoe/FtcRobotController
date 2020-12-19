package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@TeleOp(name="TeleOp", group="Linear Opmode")
public class TeleOpV1 extends LinearOpMode {
    
    private boolean[] button = {false, false, false, false, false};
    // 0 = half speed, 1 = compass, 2 = POV, 3 = launcher, 4 = launch
    private boolean[] toggle = {false, true, false, false};
    // 0 = half speed, 1 = compass, 2 = POV, 3 = launcher
    
    RobotHardware H = new RobotHardware();
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        ////////////////////////////// Init //////////////////////////////
        
        MecanumWheelDriver drive = new MecanumWheelDriver(H);
        ElapsedTime runtime = new ElapsedTime();
        ExecutorService pool = Executors.newFixedThreadPool(1);
        H.init(hardwareMap);
        
        ////////////////////////////// Init Variables //////////////////////////////
        
        double frontLeftPower;
        double frontRightPower;
        double rearLeftPower;
        double rearRightPower;
        
        double FL_RR;  //front-left and rear-right motors
        double FR_RL;  //front-right and rear-left motors
        
        double y;
        double x;
        double rotate;
        double radius;
        double stickTotal;
        double multiplier;
        double angle;
        double cosAngle;
        double sinAngle;
        double target;
        double rotateRadius;
        
        double agl_frwd = 180;
        double heading = 0;
    
        double launchStartTime = 0;
        
        waitForStart();
        runtime.reset();
        
        while (opModeIsActive()) {
            
            ////////////////////////////// Set Variables //////////////////////////////
            
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            
            if (toggle[2]) {
                
                rotateRadius = exponentialScaling(Range.clip(Math.hypot(gamepad1.right_stick_x, (-gamepad1.right_stick_y)), 0, 1));
                
                if (rotateRadius > 0.2) {
                    target = drive.addDegree(Math.toDegrees(Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x)), -90);
                    rotate = drive.POVRotate(target + agl_frwd, rotateRadius);
                } else {
                    rotate = 0;
                }
                
            } else {
                
                rotate = exponentialScaling(Range.clip(gamepad1.right_stick_x, -1, 1));
                
            }
            
            radius = exponentialScaling(Range.clip(Math.hypot(x, y), -1, 1));
            
            stickTotal = radius + Math.abs(rotate);
            angle = Math.atan2(y, x) + Math.toRadians(agl_frwd - heading - 45);
            cosAngle = Math.cos(angle);
            sinAngle = Math.sin(angle);
            
            ////////////////////////////// Mecanum Wheel Stuff //////////////////////////////
            
            if (Math.abs(cosAngle) > Math.abs(sinAngle)) {   //scale the motor's speed so that at least one of them = 1
                multiplier = 1 / Math.abs(cosAngle);
            } else {
                multiplier = 1 / Math.abs(sinAngle);
            }
    
            FL_RR = multiplier * cosAngle;
            FR_RL = multiplier * sinAngle;
            
            frontLeftPower = FL_RR * radius + rotate; //then add the rotate speed
            frontRightPower = FR_RL * radius - rotate;
            rearLeftPower = FR_RL * radius + rotate;
            rearRightPower = FL_RR * radius - rotate;
            
            if (Math.abs(stickTotal) > 1) {
                
                frontLeftPower = frontLeftPower / stickTotal;
                frontRightPower = frontRightPower / stickTotal;
                rearLeftPower = rearLeftPower / stickTotal;
                rearRightPower = rearRightPower / stickTotal;
                
            }
            
            if (toggle[0]) {
                
                H.driveMotor[0].setPower(frontLeftPower / 2);
                H.driveMotor[1].setPower(frontRightPower / 2);
                H.driveMotor[2].setPower(rearRightPower / 2);
                H.driveMotor[3].setPower(rearLeftPower / 2);
                
            } else {
                
                H.driveMotor[0].setPower(frontLeftPower);
                H.driveMotor[1].setPower(frontRightPower);
                H.driveMotor[2].setPower(rearRightPower);
                H.driveMotor[3].setPower(rearLeftPower);
                
            }
            
            ////////////////////////////// Buttons //////////////////////////////
            
            toggleButton(gamepad1.left_bumper | gamepad1.left_stick_button, 0); // half speed
            
            toggleButton(gamepad1.y, 1); // compass
            
            if (toggle[1]) {
                
                heading = H.getheading();
                
            } else {
                
                heading = agl_frwd;
                
            }
            
            toggleButton(gamepad1.x, 2); // POV
            
            toggleButton(gamepad1.right_bumper || gamepad1.b, 3); // launcher
            
            launcherLive(toggle[3]);
    
            if (gamepad1.right_trigger > 0.25 || gamepad1.a) {
                if (!button[4] || runtime.seconds() > launchStartTime + H.LAUNCH_REPEAT_DELAY) {
                    launchStartTime = runtime.seconds();
                    launch();
                    button[4] = true;
                }
            } else {
                button[4] = false;
            }
            
            if (gamepad1.start && stickTotal < 0.1) {
                
                agl_frwd = heading;
                
            }
            
            telemetry.addData("heading", H.getheading());
            telemetry.addData("rotate", rotate);
            telemetry.addData("radius", radius);
            telemetry.addData("angle", angle);
            telemetry.addData("Motors", "front-left (%.2f), front-right (%.2f), rear-right (%.2f), rear-left (%.2f)", H.driveMotor[0].getPower(), H.driveMotor[1].getPower(), H.driveMotor[2].getPower(), H.driveMotor[3].getPower());
            telemetry.addData("encoders", "front-left (%d), front-right (%d), rear-right (%d), rear-left (%d)", H.driveMotor[0].getCurrentPosition(), H.driveMotor[1].getCurrentPosition(), H.driveMotor[2].getCurrentPosition(), H.driveMotor[3].getCurrentPosition());
            telemetry.update();
            
        }
        
        pool.shutdownNow();
        
    }
    
    private void toggleButton(boolean gamepadIn, int numb) {
        
        if (gamepadIn) {
            
            if (!button[numb]) {
                
                toggle[numb] = !toggle[numb];
                button[numb] = true;
                
            }
            
        } else {
            
            button[numb] = false;
            
        }
        
    }
    
    void launcherLive(boolean active) {
        
        if (active) {
            H.launchMotor.setPower(1);
        } else {
            H.launchMotor.setPower(0);
        }
        
    }
    
    void launch() {
        
        H.launchServo.setPosition(H.LAUNCH_SERVO_MAX);
        sleep(H.LAUNCH_SERVO_DELAY);
        H.launchServo.setPosition(H.LAUNCH_SERVO_MIN);
    }
    
    double exponentialScaling(double input) {
        if (input > H.STICK_DEAD_ZONE) {
            return Math.signum(input) * ((Math.pow(H.EXP_BASE, Math.abs(input)) - 1) * (1 - H.INITIAL_VALUE) / (H.EXP_BASE - 1) + H.INITIAL_VALUE);
        } else {
            return 0;
        }
    }
    
}
