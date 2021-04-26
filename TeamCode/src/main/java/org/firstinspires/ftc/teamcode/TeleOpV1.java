package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@TeleOp(name="TeleOp", group="Linear Opmode")
public class TeleOpV1 extends LinearOpMode {
    
    private boolean[] button = {false, false, false, false, false, false, false, false, false, false};
    // 0 = half speed, 1 = compass, 2 = POV, 3 = launcher, 4 = collecting, 5 = reverse, 7 = launch, 8 = down arm pos, 9 = up arm pos
    private boolean[] toggle = {true, true, false, false, false, false, true};
    // 0 = half speed, 1 = compass, 2 = POV, 3 = launcher, 4 = collecting, 5 = reverse
    
    private double[] ARM_POSITIONS = {0.343,0.5,1.286};
    
    RobotHardware H = new RobotHardware();
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        ////////////////////////////// Init //////////////////////////////
        
        MecanumWheelDriver drive = new MecanumWheelDriver(H, this);
        ElapsedTime runtime = new ElapsedTime();
        ExecutorService pool = Executors.newFixedThreadPool(1);
        H.init(hardwareMap);
        AimBot aimBot = new AimBot(H, drive, pool, this);
        
        ////////////////////////////// Init Variables //////////////////////////////
        
        double frontLeftPower = 0;
        double frontRightPower = 0;
        double rearLeftPower = 0;
        double rearRightPower = 0;
    
        double frontLeftPowerGoal;
        double frontRightPowerGoal;
        double rearLeftPowerGoal;
        double rearRightPowerGoal;
        
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
        
        double agl_frwd = -90;
        double heading = 0;
    
        double launchStartTime = 0;
        
        int armPosition = 2;
        
        Float[] pos;
        
        drive.moveDone = false;
        
        waitForStart();
        runtime.reset();
        
        while (opModeIsActive()) {
            
            ////////////////////////////// Set Variables //////////////////////////////
            
            y = gamepad1.left_stick_y;
            x = -gamepad1.left_stick_x;
            
            if (toggle[2]) {
                
                rotateRadius = exponentialScaling(Range.clip(Math.hypot(-gamepad1.right_stick_x, gamepad1.right_stick_y), 0, 1));
                
                if (rotateRadius > 0.2) {
                    target = drive.addDegree(Math.toDegrees(Math.atan2(gamepad1.right_stick_y, -gamepad1.right_stick_x)), -90);
                    rotate = drive.POVRotate(target + agl_frwd, rotateRadius);
                } else {
                    rotate = 0;
                }
                
            } else {
                
                rotate = exponentialScaling(Range.clip(-gamepad1.right_stick_x, -1, 1));
                
            }
            
            radius = exponentialScaling(Range.clip(Math.hypot(x, y), 0, 1));
            
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
    
            frontLeftPowerGoal = FL_RR * radius + rotate;
            frontRightPowerGoal = FR_RL * radius - rotate;
            rearLeftPowerGoal = FR_RL * radius + rotate;
            rearRightPowerGoal = FL_RR * radius - rotate;
    
            if (Math.abs(stickTotal) > 1) {
        
                frontLeftPowerGoal /= stickTotal;
                frontRightPowerGoal /= stickTotal;
                rearLeftPowerGoal /= stickTotal;
                rearRightPowerGoal /= stickTotal;
        
            }
            
            frontLeftPower += powerFollow(frontLeftPower, frontLeftPowerGoal); //then add the rotate speed
            frontRightPower += powerFollow(frontRightPower, frontRightPowerGoal);
            rearLeftPower += powerFollow(rearLeftPower, rearLeftPowerGoal);
            rearRightPower += powerFollow(rearRightPower, rearRightPowerGoal);
            
            if (toggle[0]) {
                
                H.driveMotor[0].setPower(frontLeftPower * 0.8);
                H.driveMotor[1].setPower(frontRightPower * 0.8);
                H.driveMotor[2].setPower(rearRightPower * 0.8);
                H.driveMotor[3].setPower(rearLeftPower * 0.8);
                
            } else {
                
                H.driveMotor[0].setPower(frontLeftPower);
                H.driveMotor[1].setPower(frontRightPower);
                H.driveMotor[2].setPower(rearRightPower);
                H.driveMotor[3].setPower(rearLeftPower);
                
            }
            
            ////////////////////////////// Buttons //////////////////////////////
            
            toggleButton(gamepad1.left_stick_button, 0); // half speed
            
            toggleButton(gamepad1.y, 1); // compass
            
            if (toggle[1]) {
                
                heading = H.getheading();
                
            } else {
                
                heading = agl_frwd;
                
            }
            
            toggleButton(gamepad1.right_stick_button, 2); // POV
            
            toggleButton(gamepad1.right_bumper || gamepad1.b, 3); // launcher
            
            toggleButton(gamepad1.left_trigger > 0.25, 4);
            
            toggleButton(gamepad1.left_bumper, 5);
            
            toggleButton(gamepad1.x, 6);
            
            if (toggle[4]) {
                if (toggle[5]) {
                    H.collectorMotor.setPower(-1);
                } else {
                    H.collectorMotor.setPower(1);
                }
            } else {
                H.collectorMotor.setPower(0);
                toggle[5] = false;
            }
            
            if (toggle[6]) {
                H.grabberServo.setPosition(H.GRABBER_SERVO_MAX);
            } else {
                H.grabberServo.setPosition(H.GRABBER_SERVO_MIN);
            }
    
            if (gamepad1.a) {
                if (!button[7] || runtime.seconds() > launchStartTime + H.LAUNCH_REPEAT_DELAY) {
                    launchStartTime = runtime.seconds();
                    launch();
                    button[7] = true;
                }
            } else {
                button[7] = false;
            }
            
            if (gamepad1.dpad_down) {
                if (!button[8]) {
                    armPosition = Range.clip(armPosition-1, 0,2);
                    button[8] = true;
                }
            } else {
                button[8] = false;
            }
    
            if (gamepad1.dpad_up) {
                if (!button[9]) {
                    armPosition = Range.clip(armPosition+1, 0,2);
                    button[9] = true;
                }
            } else {
                button[9] = false;
            }
            
            if(H.armAngle.getVoltage() > 0.01) {
                H.armServo.setPosition(Range.clip(6 * (ARM_POSITIONS[armPosition] - H.armAngle.getVoltage()) + 0.5, 0, 1));
            } else {
                H.armServo.setPosition(0.5);
            }
            
            if (gamepad1.right_trigger > 0.25) {
                aimBot.activate();
            } else {
                aimBot.disable();
                launcherLive(toggle[3]);
            }
            
            if (gamepad1.start && stickTotal < 0.1) {
                
                agl_frwd = heading;
                
            }
    
            /*pos = aimBot.getPos();
            telemetry.addData("rotate", rotate);
            telemetry.addData("radius", radius);
            telemetry.addData("angle", Math.toDegrees(angle));
            telemetry.addData("heading", H.getheading());
            if (pos[0] != null) telemetry.addData("Pos", "X (%.1f)", pos[0]);
            if (pos[1] != null) telemetry.addData("Pos", "Y (%.1f)", pos[1]);
            telemetry.addData("Ranges", "leftTOF (%.2f), left (%.2f), rightTOF (%.2f), right (%.2f), ", H.leftTOF.getDistance(DistanceUnit.INCH), H.leftRange.getDistance(DistanceUnit.INCH), H.rightTOF.getDistance(DistanceUnit.INCH), H.rightRange.getDistance(DistanceUnit.INCH));
            telemetry.addData("Motors", "front-left (%.2f), front-right (%.2f), rear-right (%.2f), rear-left (%.2f)", H.driveMotor[0].getPower(), H.driveMotor[1].getPower(), H.driveMotor[2].getPower(), H.driveMotor[3].getPower());
            */
            //telemetry.addData("encoders", "front-left (%d), front-right (%d), rear-right (%d), rear-left (%d), launcher (%d)", H.driveMotor[0].getCurrentPosition(), H.driveMotor[1].getCurrentPosition(), H.driveMotor[2].getCurrentPosition(), H.driveMotor[3].getCurrentPosition(), H.launchMotor.getCurrentPosition());
            //telemetry.addData("servo:", H.grabberServo.getPosition());
            //telemetry.addData("volts:", H.armAngle.getVoltage());
            //telemetry.update();
            
        }
        
        pool.shutdownNow();
        aimBot.end();
        
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
            H.launchMotor.setPower(H.LAUNCH_MAX_SPEED);
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
        if (Math.abs(input) > H.STICK_DEAD_ZONE) {
            return Math.signum(input) * ((Math.pow(H.EXP_BASE, Math.abs(input)) - 1) * (1 - H.INITIAL_VALUE) / (H.EXP_BASE - 1) + H.INITIAL_VALUE);
        } else {
            return 0;
        }
    }
    
    double powerFollow(double currentPower, double goalPower) {
        if (Math.abs(goalPower - currentPower) >= H.POWER_FOLLOW_INCREMENT) {
            return Math.signum(goalPower - currentPower) * H.POWER_FOLLOW_INCREMENT;
        } else {
            return goalPower - currentPower;
        }
    }
    
}
