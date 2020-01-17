package org.firstinspires.ftc.teamcode.statemachine;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import static android.content.Context.SENSOR_SERVICE;

public class spike_StateMachineRobotBase implements SensorEventListener {

    DcMotor frontRight, frontLeft, backRight, backLeft, encoderWheel, encoderWheelHorizontal, lift, flip;

    Servo foundRight, foundLeft, mainFlop, subFlop, release;

    OpMode callingOpMode;

    private double ticksPerRotation = 488;
    private double ticksPerRotationTetrix = 1400;

    private double wheelDiameter = 4.0;

    double ticksPerInch = ((ticksPerRotation)/(wheelDiameter * Math.PI));
    double ticksPerInchTetrix = ((ticksPerRotationTetrix)/(3 * Math.PI));

    protected static final double P_DRIVE_COEFF = 0.042;

    static final double driveSpeed = 0.9;

    int frontRightBaseEncoder, frontLeftBaseEncoder, backRightBaseEncoder, encoderWheelBaseEncoder, encoderWheelHorizontalBaseEncoder, backLeftBaseEncoder = 0;

    private DistanceSensor distSensor;
    //variables for gyro operation
    private float zero;
    private float rawGyro;
    public int sensorDataCounter = 0;

    //arrays for gyro operation
    private float[] rotationMatrix = new float[9];
    private float[] orientation = new float[3];
    //objects for gyro operation
    private SensorManager mSensorManager;
    private Sensor mRotationVectorSensor;

    //DRIVE STRAIGHT MICRO STATES
    double driveStraightError;                                           //The number of degrees between the true heading and desired heading
    double driveStraightCorrection;                                      //Modifies power to account for error
    double driveStraightLeftPower;                                       //Power being fed to left side of bot
    double driveStraightRightPower;                                      //Power being fed to right side of bot
    double driveStraightMax;                                             //To be used to keep powers from exceeding 1
    double driveStraight_P_COEFF = 0.002;
    double driveStraight_D_COEFF = 0.00042;
    double driveStraightPower;
    double driveStraightDeltaT;
    double driveStraightDerivative = 0;
    double driveStraightInches;
    float driveStraightHeading;
    double driveStraightSpeedLimit;
    int driveStraightDeltaD;
    int driveStraightLastEncoder;
    int driveStraightDistance;
    int driveStraightTarget;
    boolean isStartingDriveStraight;
    double driveStraightLastTime;
    int driveStraightLoops;

    protected boolean hasBeenZeroed= false;

    float zRotation;

    int pos = 1;

    enum DriveTrainState {
        DRIVER_CONTROLLED,
        DRIVING_STRAIGHT,
        TURNING_TO_ANGLE,
        STRAFING,
        ARCING,
        STOPPED;

        boolean isStopping = false;
    }
    public class Timer {
        long startTime = 0;
        long time;
        public void startTimer(int x) {
            startTime = System.currentTimeMillis();
            time = x;
        }
        public boolean isFinished() {
            return System.currentTimeMillis() - startTime >= time;
        }
    }

    DriveTrainState currentDriveState = DriveTrainState.STOPPED;

    public spike_StateMachineRobotBase(OpMode _callingOpMode) {
        callingOpMode = _callingOpMode;

        frontRight = callingOpMode.hardwareMap.dcMotor.get("frontRight");
        frontLeft = callingOpMode.hardwareMap.dcMotor.get("frontLeft");
        backRight = callingOpMode.hardwareMap.dcMotor.get("backRight");
        backLeft = callingOpMode.hardwareMap.dcMotor.get("backLeft");
        encoderWheel = callingOpMode.hardwareMap.dcMotor.get("encoderWheelY");
        encoderWheelHorizontal = callingOpMode.hardwareMap.dcMotor.get("encoderWheelX");
        lift = callingOpMode.hardwareMap.dcMotor.get("lift");
        flip = callingOpMode.hardwareMap.dcMotor.get("flip");

        foundLeft  = callingOpMode.hardwareMap.servo.get("foundationLeft");
        foundRight = callingOpMode.hardwareMap.servo.get("foundationRight");
        mainFlop   = callingOpMode.hardwareMap.servo.get("mainFlop");
        subFlop    = callingOpMode.hardwareMap.servo.get("subFlop");
        release    = callingOpMode.hardwareMap.servo.get("release");

        distSensor = callingOpMode.hardwareMap.get(DistanceSensor.class, "distSensor");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        release.setPosition(0);
        foundRight.setPosition(0);
        foundLeft.setPosition(0.75);
        mainFlop.setPosition(0.7);
        subFlop.setPosition(0.35);

        mSensorManager = (SensorManager) _callingOpMode.hardwareMap.appContext.getSystemService(SENSOR_SERVICE);
        mRotationVectorSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR);
        mSensorManager.registerListener(this, mRotationVectorSensor, 10000);
    }
    public void stopAndReset() throws InterruptedException {
        DriveTrainState currentDriveState = DriveTrainState.STOPPED;
    }
    public void setEncoderBase(){
        frontRightBaseEncoder = frontRight.getCurrentPosition();
        frontLeftBaseEncoder = frontLeft.getCurrentPosition();
        backRightBaseEncoder = backRight.getCurrentPosition();
        backLeftBaseEncoder = backLeft.getCurrentPosition();
        encoderWheelBaseEncoder = encoderWheel.getCurrentPosition();
        encoderWheelHorizontalBaseEncoder = encoderWheel.getCurrentPosition();
    }

    public void updateDriveMotors(double frontleft, double frontright, double backleft, double backright) {
        frontRight.setPower(frontright);
        backRight.setPower(backright);
        frontLeft.setPower(frontleft);
        backLeft.setPower(backleft);
    }

    public int getCurrentFrontRightPosition() {return Math.abs(frontRight.getCurrentPosition()-frontRightBaseEncoder);}

    public int getCurrentFrontLeftPosition() {return Math.abs(frontLeft.getCurrentPosition()-frontLeftBaseEncoder);}

    public int getCurrentBackRightPosition() {return Math.abs(backRight.getCurrentPosition()-backRightBaseEncoder);}

    public int getCurrentBackLeftPosition() {return Math.abs(backLeft.getCurrentPosition()-backLeftBaseEncoder);}

    public int getCurrentAveragePosition() {return Math.abs((getCurrentBackLeftPosition()+getCurrentBackRightPosition()+getCurrentFrontRightPosition())/3);} //frontLeft is MIA

    public int getEncoderWheelPosition() {return Math.abs(encoderWheel.getCurrentPosition()-encoderWheelBaseEncoder);}

    public int getEncoderWheelHorizontalPosition() {return Math.abs(encoderWheelHorizontal.getCurrentPosition()-encoderWheelHorizontalBaseEncoder);}

    public void driveStraight(double inches, float heading, double speedLimit) {
        driveStraightInches = inches;
        driveStraightHeading = heading;
        driveStraightSpeedLimit = speedLimit;
        currentDriveState = DriveTrainState.DRIVING_STRAIGHT;
    }

    public void updateDriveTrain() {
        switch(currentDriveState) {
            case STOPPED:
                if(currentDriveState.isStopping) {
                    currentDriveState.isStopping = false;
                    frontRight.setPower(0);
                    backRight.setPower(0);
                    frontLeft.setPower(0);
                    backLeft.setPower(0);

                    frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    encoderWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    encoderWheelHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    encoderWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    encoderWheelHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                break;
            case DRIVING_STRAIGHT:
                if (isStartingDriveStraight) {
                    isStartingDriveStraight = false;
                    driveStraight_P_COEFF = 0.002;
                    driveStraight_D_COEFF = 0.00042;
                    if(driveStraightInches < 24){
                        driveStraight_P_COEFF = 0.0015;
                        driveStraight_D_COEFF = 0.00022;
                    }
                    driveStraightDerivative = 0;
                    driveStraightHeading = (int) normalize360(driveStraightHeading);
                    setEncoderBase();
                    driveStraightLastEncoder = encoderWheelBaseEncoder;
                    driveStraightTarget = (int) (driveStraightInches * ticksPerInchTetrix);
                    driveStraightSpeedLimit = Range.clip(driveStraightSpeedLimit, -1.0, 1.0);
                    if (driveStraightSpeedLimit == 0){
                        currentDriveState = DriveTrainState.STOPPED;
                    }
                    driveStraightLastTime = callingOpMode.getRuntime();
                }
                driveStraightError = driveStraightHeading - zRotation;

                driveStraightDistance = Math.abs(driveStraightTarget) - Math.abs(getEncoderWheelPosition());

                while (driveStraightError > 180) driveStraightError = (driveStraightError - 360);
                while (driveStraightError <= -180) driveStraightError = (driveStraightError + 360);

                driveStraightCorrection = Range.clip(driveStraightError * P_DRIVE_COEFF, -1, 1);

                driveStraightDeltaT = callingOpMode.getRuntime()-driveStraightLastTime;
                driveStraightLastTime = callingOpMode.getRuntime();
                driveStraightDeltaD = getEncoderWheelPosition()-driveStraightLastEncoder;
                driveStraightLastEncoder = getEncoderWheelPosition();

                driveStraightDerivative = ((double) driveStraightDeltaD)/driveStraightDeltaT;

                driveStraightPower = (driveStraightDistance*driveStraight_P_COEFF) - (driveStraightDerivative*driveStraight_D_COEFF);

                if (Math.abs(driveStraightPower) > Math.abs(driveStraightSpeedLimit)) {
                    driveStraightPower /= Math.abs(driveStraightPower);
                    driveStraightPower *= Math.abs(driveStraightSpeedLimit);
                }

                driveStraightPower *= (driveStraightSpeedLimit/Math.abs(driveStraightSpeedLimit));

                driveStraightLeftPower = driveStraightPower - driveStraightCorrection;
                driveStraightRightPower = driveStraightPower + driveStraightCorrection;

                driveStraightMax = Math.max(Math.abs(driveStraightLeftPower), Math.abs(driveStraightRightPower));
                if (driveStraightMax > 1.0) {
                    driveStraightLeftPower /= driveStraightMax;
                    driveStraightRightPower /= driveStraightMax;
                }

                updateDriveMotors(driveStraightLeftPower, driveStraightRightPower, driveStraightLeftPower, driveStraightRightPower);

                if (((driveStraightLoops+10) % 10) ==  0) {
                    callingOpMode.telemetry.addData("gyro" , zRotation);
                    callingOpMode.telemetry.addData("encoder" , getEncoderWheelPosition());
                    callingOpMode.telemetry.addData("loops", driveStraightLoops);
                    callingOpMode.telemetry.addData("deltaD", driveStraightDeltaD);
                    callingOpMode.telemetry.addData("deltaT", driveStraightDeltaT);
                    callingOpMode.telemetry.addData("distance", driveStraightDistance);
                    callingOpMode.telemetry.addData("derivative", driveStraightDerivative);
                    callingOpMode.telemetry.update();
                }

                driveStraightLoops++;

                break;
        }
    }

    public boolean isDriveStraightFinished() {
        return !(((Math.abs(driveStraightTarget)-50) > Math.abs(getEncoderWheelPosition()) || ((Math.abs(driveStraightTarget)+50) < Math.abs(getEncoderWheelPosition())))
                || (driveStraightLoops==0 || Math.abs(driveStraightDerivative)<80));
    }
    public void resetDriveStraightVars() {
        driveStraightError = 0;                                           //The number of degrees between the true heading and desired heading
        driveStraightCorrection = 0;                                      //Modifies power to account for error
        driveStraightLeftPower = 0;                                       //Power being fed to left side of bot
        driveStraightRightPower = 0;                                      //Power being fed to right side of bot
        driveStraightMax = 0;                                             //To be used to keep powers from exceeding 1
        driveStraight_P_COEFF = 0.002;
        driveStraight_D_COEFF = 0.00042;
        driveStraightPower = 0;
        driveStraightDeltaT = 0;
        driveStraightDerivative = 0;
        driveStraightInches = 0;
        driveStraightHeading = 0;
        driveStraightSpeedLimit = 0;
        driveStraightDeltaD = 0;
        driveStraightLastEncoder = 0;
        driveStraightDistance = 0;
        driveStraightTarget = 0;
        isStartingDriveStraight = false;
        driveStraightLastTime = 0;
        driveStraightLoops = 0;

    }
    @Override
    public void onSensorChanged(SensorEvent event) {
        SensorManager.getRotationMatrixFromVector(rotationMatrix, event.values);
        SensorManager.getOrientation(rotationMatrix, orientation);

        sensorDataCounter++;

        rawGyro = (float) Math.toDegrees(orientation[0]);

        //If the zero hasn't been zeroed do the zero
        if (!hasBeenZeroed) {
            hasBeenZeroed = true;
            zero = rawGyro;
        }
        //Normalize zRotation to be used
        zRotation = normalize360(rawGyro - zero);
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }

    protected float normalize360(float val) {
        while (val > 360 || val < 0) {

            if (val > 360) {
                val -= 360;
            }

            if (val < 0) {
                val += 360;
            }
        }
        return val;
    }

}
