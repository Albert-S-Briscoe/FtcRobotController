package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Thread.holdsLock;
import static java.lang.Thread.sleep;

public class RobotHardware {
    /**
     *  to use RobotHardware use this:
     *
     *      RobotHardware H = new RobotHardware();
     *
     *  then run this in runOpMode()
     *
     *      H.init(hardwareMap)
     *
     *  if you want to read a sensor value or change motor speed use this:
     *
     *      H.[sensor/motor name].[function name]([var 1], [var 2] ...);
     */

    ////////////////////////////// Constants //////////////////////////////

    public final double COUNTS_PER_REVOLUTION_CORE = 288;
    public final DcMotor.Direction[] MOTOR_DIRECTION = {DcMotor.Direction.REVERSE, DcMotor.Direction.FORWARD, DcMotor.Direction.FORWARD, DcMotor.Direction.REVERSE};
    
    public final double LAUNCH_SERVO_MAX = 0.17;
    public final double LAUNCH_SERVO_MIN = 0.02;
    public final int LAUNCH_SERVO_DELAY = 100;
    public final double LAUNCH_REPEAT_DELAY = 0.25;
    
    public final double EXP_BASE = 30;
    public final double INITIAL_VALUE = 0.05;
    public final double STICK_DEAD_ZONE = 0.05;
    
    
    ////////////////////////////// Sensors //////////////////////////////

    public DistanceSensor range;
    public BNO055IMU      imu;
    public Orientation    angles;

    ////////////////////////////// Motors //////////////////////////////
    
    public DcMotor        launchMotor;
    public DcMotor[]      driveMotor = new DcMotor[4];
    public Servo          launchServo;

    public void init(HardwareMap HM) {
        ////////////////////////////// Hardware Map //////////////////////////////

        driveMotor[0] = HM.get(DcMotor.class, "FL_Motor");
        driveMotor[1] = HM.get(DcMotor.class, "FR_Motor");
        driveMotor[2] = HM.get(DcMotor.class, "RR_Motor");
        driveMotor[3] = HM.get(DcMotor.class, "RL_Motor");
        launchMotor   = HM.get(DcMotor.class, "Launch_Motor");

        launchServo = HM.get(Servo.class, "Actuator");

        //range = HM.get(DistanceSensor.class, "range");
        imu         = HM.get(BNO055IMU.class, "imu");

        ////////////////////////////// Parameters //////////////////////////////

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile  = "BNO055IMUCalibration.json";
        imu.initialize(parameters);

        //Rev2mDistanceSensor SensorTimeOfFlight = (Rev2mDistanceSensor) range;
        
        for (int i = 3; i >= 0; i--) {
            driveMotor[i].setDirection(MOTOR_DIRECTION[i]);
            driveMotor[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotor[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        
        launchMotor.setDirection(DcMotor.Direction.FORWARD);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        launchServo.setPosition(LAUNCH_SERVO_MIN);

    }

    public double getheading() {
        // returns a value between 0 and 360
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) + 180;
    }

}
