package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class AimBot_V2 {
    
    static boolean redSide = true;
    public static int step = 0; // 0 || 3 = rotate, 1 = move x, 2 = move y, 4 = launch, 5 = disable
    public static boolean first = true;
    double launchAngle = 0;
    
    final float mmPerInch = 25.4f;
    final float mmTargetHeight = (6) * mmPerInch;
    final float halfField = 72 * mmPerInch;
    final float quadField = 36 * mmPerInch;
    final float ERROR_THRESHOLD = 6;
    final float CAMERA_OFFSET = 1.5f;
    final float ROBOT_OFFSET = 3.15f;
    final float SIDEWAYS_DISTANCE_MULTIPLIER = 1.3f;
    
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia = null;
    
    boolean targetVisible = false;
    Float[] robotPos = new Float[2];
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    VuforiaTrackables targetsUltimateGoal;
    
    MecanumWheelDriver MWD;
    RobotHardware H;
    ExecutorService pool;
    LinearOpMode TeleOp;

    AimBot_V2(RobotHardware H, MecanumWheelDriver MWD, ExecutorService pool, LinearOpMode TeleOp) {
        
        this.MWD = MWD;
        this.H = H;
        this.pool = pool;
        this.TeleOp = TeleOp;
        
        initVuforia();
        
    }
    
    public void activate() {
    
        H.launchMotor.setPower(H.LAUNCH_MAX_SPEED);
    
        Float[] pos = getPos();
        if (pos[0] == null) {
            MWD.setrotate(0, 1, true);
            MWD.rotate();
            TeleOp.sleep(100);
        }
        
        pos = getPos();
        if (pos[0] != null) {
            if (pos[1] == null) {
                launchAngle = Math.atan((pos[0] + 36)/(72)) - ROBOT_OFFSET;
            } else {
                launchAngle = Math.atan((pos[0] + 36)/(72 - pos[1])) - ROBOT_OFFSET;
            }
        }
    
        MWD.setrotate((int)launchAngle, 1, true);
        MWD.rotate();
        TeleOp.sleep(400);
        
        for (int i = 3; i > 0; i--) {
            launch();
            TeleOp.sleep(160);
        }
        H.launchMotor.setPower(0);
    
    }
    
    void disable() {
        
        step = 0;
        first = true;
        MWD.moveDone = false;
        MWD.stop = true;
        
    }
    
    void launch() {
        
        H.launchServo.setPosition(H.LAUNCH_SERVO_MAX);
        TeleOp.sleep(H.LAUNCH_SERVO_DELAY);
        H.launchServo.setPosition(H.LAUNCH_SERVO_MIN);
    }
    
    public Float[] getPos() {
    
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                targetVisible = true;
            
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }
        
        if (targetVisible) { // does Vuforia see anything?
            
            VectorF translation = lastLocation.getTranslation();
    
            if (Math.abs(H.getheading() - 180) < 10) { // is the robot facing forward if yes then use side distance sensors
                float X1;
                float X2 = translation.get(1) / mmPerInch + CAMERA_OFFSET;
                if (redSide) {
                     X1 = (float)(H.rightTOF.getDistance(DistanceUnit.INCH) - 72 + 9 + CAMERA_OFFSET);
                    robotPos[0] = avgOrIgnore(X1, X2, -36);
                } else {
                    X1 = (float)(72 - H.leftTOF.getDistance(DistanceUnit.INCH) - 9 + CAMERA_OFFSET);
                    robotPos[0] = avgOrIgnore(X1, X2, 36);
                }
                
            } else {
                robotPos[0] = translation.get(1) / mmPerInch + CAMERA_OFFSET;
            }
            
            robotPos[1] = translation.get(0) / mmPerInch + CAMERA_OFFSET;
        } else {
            
            if (Math.abs(H.getheading() - 180) < 10) {
                if (redSide) {
                    robotPos[0] = (float)(H.rightTOF.getDistance(DistanceUnit.INCH) - 72 + 8.5 + CAMERA_OFFSET);
                } else {
                    robotPos[0] = (float)(72 - H.leftTOF.getDistance(DistanceUnit.INCH) - 8.5 + CAMERA_OFFSET);
                }
            
            } else {
                robotPos[0] = null;
            }
            robotPos[1] = null;
            
        }
        
        return robotPos;
        
    }
    
    private Float avgOrIgnore(float X1, float X2, float goal) {
        
        if (Math.abs(X1 - X2) < ERROR_THRESHOLD) {
            return (X1 + X2) / 2;
        } else {
            if (Math.abs(X1 - goal) < Math.abs(X2 - goal)) {
                return X1;
            } else {
                return X2;
            }
        }
    }
    
    void initVuforia() {
    
        int cameraMonitorViewId = H.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", H.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        
        parameters.vuforiaLicenseKey = H.VUFORIA_KEY;
        parameters.cameraName = H.webcam;
        //parameters.cameraDirection   = CAMERA_CHOICE;
        parameters.useExtendedTracking = false;
    
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    
        targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");
        
        allTrackables.addAll(targetsUltimateGoal);
    
        //Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
    
        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));
    
        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        
        /*redAllianceTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
        
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 180)));
    
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 0)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));*/
    
        targetsUltimateGoal.activate();
        
    }
    
    public void end() {
        targetsUltimateGoal.deactivate();
    }
    
}
