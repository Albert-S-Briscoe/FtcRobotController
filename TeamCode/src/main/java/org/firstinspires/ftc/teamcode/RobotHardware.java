package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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

    ////////////////////////////// Constance //////////////////////////////

    
    final double COUNTS_PER_REVOLUTION_CORE = 288;
    
    ////////////////////////////// Sensors //////////////////////////////

    public DistanceSensor upperRange;
    public DistanceSensor lowerRange;
    public DistanceSensor sensorRange;
    public AnalogInput    vertpos;
    public DigitalChannel limit;
    public BNO055IMU      imu;
    public Orientation    angles;

    ////////////////////////////// Motors //////////////////////////////
    
    public DcMotor        vertical;
    public DcMotor[]      driveMotor = new DcMotor[4];
    public Servo[]        driveServo = new Servo[4];
    public AnalogInput[]  driveAngle = new AnalogInput[4];
    public Servo          grabber;
    public Servo          Vertical;
    public Servo          launcherServo;
    public Servo          centerServo;
    public Servo          L;
    public Servo          R;

    public void init(HardwareMap HM) {
        ////////////////////////////// Hardware Map //////////////////////////////

        //driveMotor[0] = HM.get(DcMotor.class, "FL_Motor");
        driveMotor[1] = HM.get(DcMotor.class, "FR_Motor");
        //driveMotor[2] = HM.get(DcMotor.class, "RR_Motor");
        //driveMotor[3] = HM.get(DcMotor.class, "RL_Motor");
    
        //driveServo[0] = HM.get(Servo.class, "FL_Servo");
        //driveServo[1] = HM.get(Servo.class, "FR_Servo");
        //driveServo[2] = HM.get(Servo.class, "RR_Servo");
        //driveServo[3] = HM.get(Servo.class, "RL_Servo");
    
        //driveAngle[0] = HM.get(AnalogInput.class, "FL_Angle");
        //driveAngle[1] = HM.get(AnalogInput.class, "FR_Angle");
        //driveAngle[2] = HM.get(AnalogInput.class, "RR_Angle");
        //driveAngle[3] = HM.get(AnalogInput.class, "RL_Angle");

        //grabber     = HM.get(Servo.class, "grabber");  // 1 = open, 0 = closed
        launcherServo = HM.get(Servo.class, "actuator");
        //centerServo = HM.get(Servo.class, "center");
        //vertical    = HM.get(DcMotor.class, "vertical");
        //L           = HM.get(Servo.class, "GrabberLeft");
        //R           = HM.get(Servo.class, "GrabberRight");

        //vertpos     = HM.get(AnalogInput.class, "vert_pos");
        //upperRange  = HM.get(DistanceSensor.class, "upper_range");
        //lowerRange  = HM.get(DistanceSensor.class, "lower_range");
        //sensorRange = HM.get(DistanceSensor.class, "sensor_range");
        imu         = HM.get(BNO055IMU.class, "imu");

        ////////////////////////////// Parameters //////////////////////////////

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile  = "BNO055IMUCalibration.json";
        imu.initialize(parameters);

        Rev2mDistanceSensor SensorTimeOfFlight1 = (Rev2mDistanceSensor) upperRange;
        Rev2mDistanceSensor SensorTimeOfFlight2 = (Rev2mDistanceSensor) lowerRange;
        Rev2mDistanceSensor SensorTimeOfFlight3 = (Rev2mDistanceSensor) sensorRange;
    
        //driveMotor[0].setDirection(DcMotor.Direction.FORWARD);
        driveMotor[1].setDirection(DcMotor.Direction.REVERSE);
        //driveMotor[2].setDirection(DcMotor.Direction.FORWARD);
        //driveMotor[3].setDirection(DcMotor.Direction.REVERSE);
    
        //driveMotor[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveMotor[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //driveMotor[2].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //driveMotor[3].setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //vertical.setDirection(DcMotor.Direction.REVERSE);
        //vertical.setTargetPosition(0);
        //vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public double getheading() {
        // returns a value between 0 and 360
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) + 180;

    }

    public void grab(boolean down) {

        if (down) {
            L.setPosition(1);
            R.setPosition(0);
        } else {
            L.setPosition(0);
            R.setPosition(1);
        }

    }

    public void block(boolean closed) {

        if (closed) {
            centerServo.setPosition(0.9);
            grabber.setPosition(0);
        } else {
            centerServo.setPosition(0.4);
            grabber.setPosition(1);
        }

    }

}
