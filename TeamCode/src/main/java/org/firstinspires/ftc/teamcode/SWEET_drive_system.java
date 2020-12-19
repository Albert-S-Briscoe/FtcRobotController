package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class SWEET_drive_system {
    
    
    ExecutorService pool = Executors.newFixedThreadPool(1);
    RobotHardware H;
    Wheel[] wheel = new Wheel[4];
    
    final double S1 = 6;
    final double S2 = 2;
    
    static boolean pauseToTurn = false;
    double powerLinear;
    double powerRotate;
    double powerAngle;
    
    SWEET_drive_system(RobotHardware H) {
        this.H = H;
        //wheel[0] = new Wheel(1,1, H.driveMotor[0], H.driveServo[0], H.driveAngle[0]);
        //wheel[1] = new Wheel(-1,1, H.driveMotor[1], H.driveServo[1], H.driveAngle[1]);
        //wheel[2] = new Wheel(-1, -1, H.driveMotor[2], H.driveServo[2], H.driveAngle[2]);
        //wheel[3] = new Wheel(1,-1, H.driveMotor[3], H.driveServo[3], H.driveAngle[3]);
    }
    
    void moveArc(double power, double angle, double rotatePower, double angleForwards) {
        powerLinear = power;
        powerAngle = angle;
        wheel[0].setMove(angle,power);
        if (pauseToTurn) wheel[0].setPower(0);
        pool.execute(wheel[0]);
        powerLinear = power;
        powerAngle = angle - angleForwards;
    }
    
    void moveArc(double power, double angle,  double rotatePower) {
        double[] wheelRadii = new double[4];
        double[] wheelPower = new double[4];
        double[] wheelAngle = new double[4];
        final double radiiMax;
        final double wheelMax;
        final double pointAngle;
        final double pointDistance;
        final double X;
        final double Y;
        
        if (rotatePower == 0) rotatePower += 0.00001;
        wheelMax = Math.max(Math.abs(rotatePower), Math.abs(power));
        pointAngle = Math.toRadians(angle - 90* Math.signum(-rotatePower));
        pointDistance = Math.abs(100 * Math.pow(power/(S1 * -rotatePower),S2));
        X = Math.cos(pointAngle) * pointDistance;
        Y = Math.sin(pointAngle) * pointDistance;
        
        for (int i = 0; i <= 3; i++) {
            wheelRadii[i] = Math.hypot(X - wheel[i].x, Y - wheel[i].y);
        }
        
        radiiMax = Math.max(Math.max(wheelRadii[0], wheelRadii[1]), Math.max(wheelRadii[2], wheelRadii[3]));
        
        for (int i = 0; i <= 3; i++) {
            wheelPower[i] = wheelMax * wheelRadii[i]/radiiMax;
            wheelAngle[i] = Math.atan2(wheel[i].x - X, wheel[i].y - Y) + Math.PI/2 * Math.signum(-rotatePower);
        }
        
    }
    
    void move(double power, double angle, double inches) {
    
    }
    
    void move(double power, double angle, double inches, double angleForwards) {
    
    }
    
    void rotate(double power) {
        powerRotate = power;
    }
    
    void rotate(double power, double angle, boolean fieldRelative) {
    
    }
    
    static double addDegree(double DegCurrent, double addDeg) {
        
        /**adds a number of degrees to the current degree with rapping around from 360 to 0
         * returns a value between 0 and 360
         */
        
        double output = DegCurrent + addDeg;
        while (output < 0 || output > 360) {
            if (output >= 360) {
                output -= 360;
            } else if (output < 0) {
                output += 360;
            }
        }
        return output;
    }
    
    static double FindDegOffset(double DegCurrent, double TargetDeg) {
        
        /**DegCurrent, the current degree of the robot value between 0 and 360
         * TargetDeg, the degree with which to find the offset
         * Finds the angle between current degree and the target degree
         * returns a value between -180 and 180
         * output will be negative if the current degree is left of the target, positive if on the right
         *    0
         * 90   -90
         *   180
         */
        
        double offset = TargetDeg - DegCurrent;
        if (offset > 180) {
            offset -= 360;
        } else if (offset < -180) {
            offset += 360;
        }
        return offset;
    }
    
    void end() {
        wheel[0].stop();
        pool.shutdownNow();
    }
    
}

class Wheel implements Runnable {
    
    static final double VOLTS = 3.3;
    static final int DEAD_ZONE_SIZE = -24;
    static final double DEGREES_PER_VOLT = (180 - DEAD_ZONE_SIZE)/VOLTS;
    final int PAUSE_TO_TURN_THRESHOLD_ANGLE = 45;
    final int ANGLE_TOLERANCE = 4;
    final double DEGREES_OF_DECELERATION = 30;
    final double TICK_SIZE = 0.1;
    final double COUNTS_PER_REVOLUTION = 112;
    final double COUNTS_PER_DEGREE = COUNTS_PER_REVOLUTION/360;
    final double TIME_PER_DEGREE = 0.0378889;
    final int x;
    final int y;
    final DcMotor driveMotor;
    final Servo rotateMotor;
    final AnalogInput angleInput;
    
    int wheelDirection = 1;
    double currentDegree = 0;
    double destinationAngle;
    double power;
    double angle;
    double previousAngle;
    
    Wheel(final int x, final int y, final DcMotor driveMotor, final Servo rotateMotor, final AnalogInput angleInput) {
        this.x = x;
        this.y = y;
        this.driveMotor = driveMotor;
        this.rotateMotor = rotateMotor;
        this.angleInput = angleInput;
        driveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    
    @Override
    public void run() {
        /*int degreesIntoUnknown;
        int offset = rotate(angle);
        if (latestDestinationDegree >= 180) latestDestinationDegree -= 180;
        if (latestDestinationDegree <= DEAD_ZONE_SIZE) {
            if (offset > 0) degreesIntoUnknown = DEAD_ZONE_SIZE - (int) latestDestinationDegree;
            else degreesIntoUnknown = (int) latestDestinationDegree;
        } else degreesIntoUnknown = -1;
        /*if (SWEET_drive_system.pauseToTurn) {
            driveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveMotor.setPower(0.2);
            driveMotor.setTargetPosition((int)(offset * COUNTS_PER_REVOLUTION));
        }*/
        /*while (currentDegree > latestDestinationDegree + ANGLE_TOLERANCE || currentDegree < latestDestinationDegree - ANGLE_TOLERANCE) {
            currentDegree = angleInput.getVoltage() * DEGREES_PER_VOLT + DEAD_ZONE_SIZE;
            if (currentDegree <= DEAD_ZONE_SIZE + 4 && degreesIntoUnknown != -1) {
                //rotateMotor.setPosition(-Math.signum(offset) * 0.15 + 0.5);
                try {
                    wait((int) ((double)degreesIntoUnknown * TIME_PER_DEGREE * 1000));
                } catch (Exception e) {
                
                }
                rotateMotor.setPosition(0.5);
                break;
            }
            rotateMotor.setPosition(-Math.signum(offset) * Math.abs(Range.clip( SWEET_drive_system.FindDegOffset(currentDegree, latestDestinationDegree)/DEGREES_OF_DECELERATION, -1, 1)/2) + 0.5001);
        }
        rotateMotor.setPosition(0.5);
        //driveMotor.setTargetPosition(driveMotor.getCurrentPosition());
        //driveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //setPower(power);*/
        double offset;
        destinationAngle = 180 + angle;
        if (destinationAngle >= 180) destinationAngle -= 180;
        if (destinationAngle <= DEAD_ZONE_SIZE) destinationAngle = 180;
        currentDegree = angleInput.getVoltage() * DEGREES_PER_VOLT + DEAD_ZONE_SIZE;
        offset = FindDegOffset(currentDegree, destinationAngle, 180);
        double stickOffset = FindDegOffset(previousAngle + 180, angle + 180, 360);
        if (Math.signum(offset) * Math.signum(stickOffset) == -1 || Math.abs(stickOffset) > 105 ) {
            wheelDirection = -wheelDirection;
        }
        previousAngle = angle;
        
        if (Math.abs(offset) > ANGLE_TOLERANCE) {
            driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveMotor.setTargetPosition((int) (offset * COUNTS_PER_DEGREE) + driveMotor.getCurrentPosition());
            driveMotor.setPower(0.3);
            do {
                //if (voltage <= 0.1) {
                //    currentDegree = SWEET_drive_system.addDegree(currentDegree, Math.signum(FindDegOffset(currentDegree, latestDestinationDegree)) * TICK_SIZE);
                //} else {
                currentDegree = angleInput.getVoltage() * DEGREES_PER_VOLT + DEAD_ZONE_SIZE;
                //}
                offset = FindDegOffset(currentDegree, destinationAngle, 180);
                rotateMotor.setPosition(Range.clip(-offset / DEGREES_OF_DECELERATION, -1, 1) / 2 + 0.5001);
            } while (Math.abs(offset) > ANGLE_TOLERANCE);
            rotateMotor.setPosition(0.5);
            driveMotor.setTargetPosition(driveMotor.getCurrentPosition());
            driveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        driveMotor.setPower(power * wheelDirection);
    }
    
    void setMove(double angle, double power) {
        this.power = power;
        this.angle = angle;
    }
    
    int rotate(double angle) {
        int offset = (int)SWEET_drive_system.FindDegOffset(destinationAngle, angle);
        SWEET_drive_system.pauseToTurn = Math.abs(offset) > PAUSE_TO_TURN_THRESHOLD_ANGLE && !SWEET_drive_system.pauseToTurn;
        if (Math.abs(offset) > 90) {
            offset -= 180 * Math.signum(offset);
            wheelDirection = -wheelDirection;
        }
        destinationAngle = SWEET_drive_system.addDegree(destinationAngle, offset);
        return offset;
    }
    
    void setPower(double power) {
        driveMotor.setPower(power * wheelDirection);
    }
    
    void setMoveDistance(double inches, boolean reletiveToStart) {
        driveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveMotor.setTargetPosition((int)(inches));
    }
    
    void stop() {
        driveMotor.setPower(0);
    }
    
    static double FindDegOffset(double DegCurrent, double TargetDeg, int angleSize) {
        
        /**DegCurrent, the current degree. value between 0 and 180
         * TargetDeg, the degree with which to find the offset
         * Finds the angle between current degree and the target degree
         * returns a value between -90 and 90
         * output will be negative if the target is on the right, positive if on the left
         *    0
         * 90   -90
         *   180
         */
        
        double offset = TargetDeg - DegCurrent;
        if (offset > (float)angleSize/2) {
            offset -= angleSize;
        } else if (offset < -(float)angleSize/2) {
            offset += angleSize;
        }
        return offset;
    }
    
}
