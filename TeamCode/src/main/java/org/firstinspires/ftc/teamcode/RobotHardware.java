package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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
    
    public HardwareMap hardwareMap;

    ////////////////////////////// Constants //////////////////////////////

    public final double COUNTS_PER_REVOLUTION_CORE = 288;
    public final DcMotor.Direction[] MOTOR_DIRECTION = {DcMotor.Direction.REVERSE, DcMotor.Direction.FORWARD, DcMotor.Direction.FORWARD, DcMotor.Direction.REVERSE};
    
    public final double LAUNCH_SERVO_MAX = 0.17;
    public final double LAUNCH_SERVO_MIN = 0.02;
    public final int LAUNCH_SERVO_DELAY = 100;
    public final double LAUNCH_REPEAT_DELAY = 0.25;
    public final double LAUNCH_MAX_SPEED = 0.92;
    
    public final double EXP_BASE = 20;
    public final double INITIAL_VALUE = 0.05;
    public final double STICK_DEAD_ZONE = 0.1;
    
    public final String VUFORIA_KEY = "AXfJetz/////AAABmfTftTQRKUq2u+iCzbuFm2wKhp5/qubTF+6xF9VBwMBiVi2lCwJbNrIAVofnUKke4/MjFtZROHGeelAgbQx6MjYX+qdX4vRB5z2PboepftoqvoZy3irQKQ2aKqNSbpN72hI/tI2wluN0xqC6KThtMURH0EuvUf8VcGDfmuXiA/uP00/2dsYhIMhxBJCmBq0AG5jMWi8MnHJDZwnoYLdcliKB7rvNTUDbf1fzxRzf9QHgB2u+invzPou7q8ncAsD5GdXFfA/CiYmR65JKXDOE0wHoc8FxvrzUIRCQ2geSypo7eY5q/STJvqPmjoj33CQFHl0hKMx05QwwsABdlIZvfLLbjA3VH2HO4dcv+OOoElws";
    
    
    ////////////////////////////// Sensors //////////////////////////////

    public DistanceSensor leftRange;
    public DistanceSensor rightRange;
    public Rev2mDistanceSensor leftTOF;
    public Rev2mDistanceSensor rightTOF;
    public BNO055IMU      imu;
    public Orientation    angles;
    public WebcamName     webcam;

    ////////////////////////////// Motors //////////////////////////////
    
    public DcMotor        launchMotor;
    public DcMotor        collectorMotor;
    public DcMotor[]      driveMotor = new DcMotor[4];
    public Servo          launchServo;

    public void init(HardwareMap HM) {
        
        hardwareMap = HM;
        
        ////////////////////////////// Hardware Map //////////////////////////////

        driveMotor[0] = HM.get(DcMotor.class, "FL_Motor");
        driveMotor[1] = HM.get(DcMotor.class, "FR_Motor");
        driveMotor[2] = HM.get(DcMotor.class, "RR_Motor");
        driveMotor[3] = HM.get(DcMotor.class, "RL_Motor");
        launchMotor   = HM.get(DcMotor.class, "Launch_Motor");
        collectorMotor   = HM.get(DcMotor.class, "Collector_Motor");

        launchServo = HM.get(Servo.class, "Actuator");
    
        webcam = HM.get(WebcamName.class, "Webcam 1");
        leftRange = HM.get(DistanceSensor.class, "L_Range");
        rightRange = HM.get(DistanceSensor.class, "R_Range");
        imu = HM.get(BNO055IMU.class, "imu");

        ////////////////////////////// Parameters //////////////////////////////

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile  = "BNO055IMUCalibration.json";
        imu.initialize(parameters);

        leftTOF = (Rev2mDistanceSensor) leftRange;
        rightTOF = (Rev2mDistanceSensor) rightRange;
        
        for (int i = 3; i >= 0; i--) {
            driveMotor[i].setDirection(MOTOR_DIRECTION[i]);
            driveMotor[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotor[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveMotor[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        
        launchMotor.setDirection(DcMotor.Direction.FORWARD);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        launchServo.setPosition(LAUNCH_SERVO_MIN);

    }

    public double getheading() {
        // returns a value between 0 and 360
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) + 180;
    }

}
