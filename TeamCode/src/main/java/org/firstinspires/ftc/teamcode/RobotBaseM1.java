package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import static android.content.Context.SENSOR_SERVICE;

@SuppressWarnings({"WeakerAccess", "FieldCanBeLocal"})
public class RobotBaseM1 implements SensorEventListener {

    DcMotor frontRight, frontLeft, backRight, backLeft, lift, flip;

    Servo foundRight, foundLeft, mainFlop, subFlop, release;

    OpMode callingOpMode;

    private double ticksPerRotation = 488;

    private double wheelDiameter = 4;

    double ticksPerInch = ((ticksPerRotation)/(wheelDiameter * Math.PI));

    protected static final double P_DRIVE_COEFF = 0.02;

    static final double driveSpeed = 0.9;

    int frontRightBaseEncoder, frontLeftBaseEncoder, backRightBaseEncoder, backLeftBaseEncoder = 0;

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

    protected boolean hasBeenZeroed= false;

    float zRotation;

    int pos = 1;

    public RobotBaseM1 (OpMode _callingOpMode) {
        callingOpMode = _callingOpMode;

        frontRight = callingOpMode.hardwareMap.dcMotor.get("frontRight");
        frontLeft = callingOpMode.hardwareMap.dcMotor.get("frontLeft");
        backRight = callingOpMode.hardwareMap.dcMotor.get("backRight");
        backLeft = callingOpMode.hardwareMap.dcMotor.get("backLeft");
        lift = callingOpMode.hardwareMap.dcMotor.get("lift");
        flip = callingOpMode.hardwareMap.dcMotor.get("flip");

        foundLeft  = callingOpMode.hardwareMap.servo.get("foundationLeft");
        foundRight = callingOpMode.hardwareMap.servo.get("foundationRight");
        mainFlop   = callingOpMode.hardwareMap.servo.get("mainFlop");
        subFlop    = callingOpMode.hardwareMap.servo.get("subFlop");
        release    = callingOpMode.hardwareMap.servo.get("release");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);


        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        release.setPosition(0);
        foundRight.setPosition(0.75);
        foundLeft.setPosition(0);
        mainFlop.setPosition(0.7);
        subFlop.setPosition(0.35);

        mSensorManager = (SensorManager) _callingOpMode.hardwareMap.appContext.getSystemService(SENSOR_SERVICE);
        mRotationVectorSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR);
        mSensorManager.registerListener(this, mRotationVectorSensor, 10000);
    }

    public void updateDriveMotors(double frontleft, double frontright, double backleft, double backright) {
        frontRight.setPower(frontright);
        backRight.setPower(backright);
        frontLeft.setPower(frontleft);
        backLeft.setPower(backleft);
    }

    public void stopAndReset() throws InterruptedException {

        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);

        Thread.sleep(200);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Thread.sleep(10);
    }

    public void setEncoderBase(){
        frontRightBaseEncoder = frontRight.getCurrentPosition();
        frontLeftBaseEncoder = frontLeft.getCurrentPosition();
        backRightBaseEncoder = backRight.getCurrentPosition();
        backLeftBaseEncoder = backLeft.getCurrentPosition();
    }

    public int getCurrentFrontRightPosition() {return Math.abs(frontRight.getCurrentPosition()-frontRightBaseEncoder);}

    public int getCurrentFrontLeftPosition() {return Math.abs(frontLeft.getCurrentPosition()-frontLeftBaseEncoder);}

    public int getCurrentBackRightPosition() {return Math.abs(backRight.getCurrentPosition()-backRightBaseEncoder);}

    public int getCurrentBackLeftPosition() {return Math.abs(backLeft.getCurrentPosition()-backLeftBaseEncoder);}

    public int getCurrentAveragePosition() {return Math.abs((getCurrentBackLeftPosition()+getCurrentBackRightPosition()+getCurrentFrontRightPosition())/3);} //frontLeft is MIA

    public void driveStraight(double inches, float heading) throws InterruptedException { driveStraight(inches, heading, driveSpeed); }

    public void driveStraight(double inches, float heading, double power)  throws InterruptedException {
        double error;                                           //The number of degrees between the true heading and desired heading
        double correction;                                      //Modifies power to account for error
        double leftPower;                                       //Power being fed to left side of bot
        double rightPower;                                      //Power being fed to right side of bot
        double max;                                             //To be used to keep powers from exceeding 1
        long loops = 0;
        heading = (int) normalize360(heading);

        setEncoderBase();

        int target = (int) (inches * ticksPerInch);

        power = Range.clip(power, -1.0, 1.0);


        while (Math.abs(target) > Math.abs(getCurrentAveragePosition())  && ((LinearOpMode) callingOpMode).opModeIsActive()) {

            error = heading - zRotation;

            while (error > 180) error = (error - 360);
            while (error <= -180) error = (error + 360);

            correction = Range.clip(error * P_DRIVE_COEFF, -1, 1);

            leftPower = power - correction;
            rightPower = power + correction;

            max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > 1.0) {
                leftPower /= max;
                rightPower /= max;
            }
            updateDriveMotors(leftPower, rightPower, leftPower, rightPower);

            if (((loops+10) % 10) ==  0) {
                callingOpMode.telemetry.addData("gyro" , zRotation);
                callingOpMode.telemetry.addData("encoder" , getCurrentAveragePosition());
                callingOpMode.telemetry.addData("loops", loops);
                callingOpMode.telemetry.update();
            }

            loops++;

            Thread.yield();
        }
    }

    public void driveStraightTime(long millis, float heading, double power)  throws InterruptedException {
        double error;                                           //The number of degrees between the true heading and desired heading
        double correction;                                      //Modifies power to account for error
        double leftPower;                                       //Power being fed to left side of bot
        double rightPower;                                      //Power being fed to right side of bot
        double max;                                             //To be used to keep powers from exceeding 1
        long loops = 0;
        heading = (int) normalize360(heading);

        setEncoderBase();

        power = Range.clip(power, -1.0, 1.0);

        long startTime = System.currentTimeMillis();


        while ((System.currentTimeMillis() < (startTime+millis))  && ((LinearOpMode) callingOpMode).opModeIsActive()) {

            error = heading - zRotation;

            while (error > 180) error = (error - 360);
            while (error <= -180) error = (error + 360);

            correction = Range.clip(error * P_DRIVE_COEFF, -1, 1);

            leftPower = power - correction;
            rightPower = power + correction;

            max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > 1.0) {
                leftPower /= max;
                rightPower /= max;
            }
            updateDriveMotors(leftPower, rightPower, leftPower, rightPower);

            if (((loops+10) % 10) ==  0) {
                callingOpMode.telemetry.addData("gyro" , zRotation);
                callingOpMode.telemetry.addData("encoder" , getCurrentAveragePosition());
                callingOpMode.telemetry.addData("loops", loops);
                callingOpMode.telemetry.update();
            }

            loops++;

            Thread.yield();
        }
    }

    public void turn(float turnHeading, double power) throws InterruptedException {
        int wrapFix = 0;                                        //Can be used to modify values and make math around 0 easier
        float shiftedTurnHeading = turnHeading;                 //Can be used in conjunction with wrapFix to make math around 0 easier
        long loops = 0;

        power = Math.abs(power);                                //makes sure the power is positive
        if (power>1) power = 1;                                 //makes sure the power isn't >1

        //If heading is not on correct scale, put it between 0-360
        turnHeading = normalize360(turnHeading);

        //Figure out how far the robot would have to turn in counterclockwise & clockwise directions
        float cclockwise = zRotation - turnHeading;
        float clockwise = turnHeading - zRotation;

        //Normalize cwise & ccwise values to between 0=360
        clockwise = normalize360(clockwise);
        cclockwise = normalize360(cclockwise);
        int error = 1;

        //sets the distance to the target gyro value that we will accept
        if (turnHeading - error < 0|| turnHeading + error > 360) {
            wrapFix = 180;                                      //if within the range where the clockmath breaks, shift to an easier position
            shiftedTurnHeading = normalize360(turnHeading + wrapFix);
        }

        //If it would be longer to take the ccwise path, we go *** CLOCKWISE ***
        if(Math.abs(cclockwise) >= Math.abs(clockwise)){

            updateDriveMotors(-power, power, -power, power);

            //While we're not within our error, and we haven't overshot, and the bot is running
            while(Math.abs(normalize360(zRotation + wrapFix)- shiftedTurnHeading) > error &&
                    Math.abs(cclockwise) >= Math.abs(clockwise) && ((LinearOpMode) callingOpMode).opModeIsActive()) {

                //Figure out how far the robot would have to turn in counterclockwise & clockwise directions
                cclockwise = zRotation - turnHeading;
                clockwise = turnHeading - zRotation;

                //Normalize cwise & ccwise values to between 0=360
                clockwise = normalize360(clockwise);
                cclockwise = normalize360(cclockwise);

                if ((loops % 10) ==  0) {
                    callingOpMode.telemetry.addData("gyro" , zRotation);
                    callingOpMode.telemetry.addData("loops", loops);
                    callingOpMode.telemetry.update();
                }

                loops++;

                //Chill a hot decisecond
                Thread.sleep(10);
            }
        }
        //If it would take longer to take the cwise path, we go *** COUNTERCLOCKWISE ***
        else if(Math.abs(clockwise) > Math.abs(cclockwise)) {

            updateDriveMotors(power, -power, power, -power);

            //While we're not within our error, and we haven't overshot, and the bot is running
            while (Math.abs(normalize360(zRotation + wrapFix) - shiftedTurnHeading) > error &&
                    Math.abs(clockwise) > Math.abs(cclockwise) && ((LinearOpMode) callingOpMode).opModeIsActive()) {

                //Figure out how far the robot would have to turn in counterclockwise & clockwise directions
                cclockwise = zRotation - turnHeading;
                clockwise = turnHeading - zRotation;

                //Normalize cwise & ccwise values to between 0=360
                clockwise = normalize360(clockwise);
                cclockwise = normalize360(cclockwise);

                if ((loops % 10) ==  0) {
                    callingOpMode.telemetry.addData("gyro" , zRotation);
                    callingOpMode.telemetry.addData("loops", loops);
                    callingOpMode.telemetry.update();
                }

                loops++;

                //Hold up a hot decisecond
                Thread.sleep(10);
            }
        }
    }

    public void strafe(double inches, float heading, double power)  throws InterruptedException {
        double error;                                           //The number of degrees between the true heading and desired heading
        double correction;                                      //Modifies power to account for error
        double frontPower;                                       //Power being fed to front side of bot
        double backPower;                                      //Power being fed to back side of bot
        double max;                                             //To be used to keep powers from exceeding 1
        long loops = 0;
        heading = (int) normalize360(heading);

        int target = (int) (inches * ticksPerInch);

        power = Range.clip(power, -1.0, 1.0);

        setEncoderBase();

        while (Math.abs(target) > getCurrentAveragePosition()  && ((LinearOpMode) callingOpMode).opModeIsActive()) {

            error = heading - zRotation;

            while (error > 180) error = (error - 360);
            while (error <= -180) error = (error + 360);

            correction = Range.clip(error * P_DRIVE_COEFF, -1, 1);

            frontPower = power + correction;
            backPower = power - correction;

            max = Math.max(Math.abs(frontPower), Math.abs(backPower));
            if (max > 1.0) {
                backPower /= max;
                frontPower /= max;
            }

            if ((loops % 10) ==  0) {
                callingOpMode.telemetry.addData("gyro" , zRotation);
                callingOpMode.telemetry.addData("encoder" , getCurrentAveragePosition());
                callingOpMode.telemetry.addData("loops", loops);
                callingOpMode.telemetry.update();
            }

            updateDriveMotors(-frontPower, frontPower, backPower, -backPower);

            loops++;

            Thread.yield();
        }
    }

    public void arcDrive (double inches, float degreesToTurn, double basePower, boolean clockwise, double p_coeff) throws InterruptedException {

        //degrees to turn  positive for clockwise

        int target = (int) (inches * ticksPerInch);
        double correction;
        double leftPower = basePower;
        double rightPower = basePower;
        double max;
        int lastEncoder;
        int currentEncoder = 0;
        int deltaEncoder;
        double proportionEncoder;
        float lastHeading;
        float currentHeading = zRotation;
        float deltaHeading;
        double proportionHeading;
        double proportionArc;
        long loops = 0;

        if(basePower<0){
            clockwise = !clockwise;
            degreesToTurn *= -1;
        }

        double P_ARC_COEFF = p_coeff;

        setEncoderBase();

        frontRight.setPower(basePower);
        backRight.setPower(basePower);
        frontLeft.setPower(basePower);
        backLeft.setPower(basePower);

        Thread.sleep(10);

        while (((LinearOpMode) callingOpMode).opModeIsActive()) {

            lastEncoder = currentEncoder;
            currentEncoder = getCurrentAveragePosition();

            deltaEncoder = currentEncoder - lastEncoder;

            lastHeading = currentHeading;
            currentHeading = zRotation;

            deltaHeading = currentHeading - lastHeading;
            deltaHeading = normalize360two(deltaHeading);

            proportionEncoder = (double) deltaEncoder / target;
            proportionHeading = (double) deltaHeading / degreesToTurn;

            proportionArc = proportionEncoder - proportionHeading;

            correction = proportionArc * P_ARC_COEFF;

            if(clockwise) {
                leftPower += correction;
                rightPower -= correction;
            } else {
                leftPower -= correction;
                rightPower += correction;
            }

            max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > 1.0) {
                leftPower /= max;
                rightPower /= max;
            }
            frontRight.setPower(rightPower);
            backRight.setPower(rightPower);
            frontLeft.setPower(leftPower);
            backLeft.setPower(leftPower);

            if (((loops + 10) % 10) == 0) {
                callingOpMode.telemetry.addData("TARGET", target);
                callingOpMode.telemetry.addData("gyro", zRotation);
                callingOpMode.telemetry.addData("encoder", currentEncoder);
                callingOpMode.telemetry.addData("heading", currentHeading);
                callingOpMode.telemetry.addData("dEncoder", deltaEncoder);
                callingOpMode.telemetry.addData("dHeading", deltaHeading);
                callingOpMode.telemetry.addData("pEncoder", proportionEncoder);
                callingOpMode.telemetry.addData("pHeading", proportionHeading);
                callingOpMode.telemetry.addData("arc", proportionArc);
                callingOpMode.telemetry.addData("correction", correction);
                callingOpMode.telemetry.addData("leftPower", leftPower);
                callingOpMode.telemetry.addData("rightPower", rightPower);
                callingOpMode.telemetry.addData("loops", loops);
                callingOpMode.telemetry.update();
            }

            loops++;

            Thread.sleep(10);

            if(Math.abs(target) <= Math.abs(getCurrentAveragePosition())) {
                callingOpMode.telemetry.addData("TARGET", target);
                callingOpMode.telemetry.addData("gyro", zRotation);
                callingOpMode.telemetry.addData("encoder", currentEncoder);
                callingOpMode.telemetry.addData("heading", currentHeading);
                callingOpMode.telemetry.addData("dEncoder", deltaEncoder);
                callingOpMode.telemetry.addData("dHeading", deltaHeading);
                callingOpMode.telemetry.addData("pEncoder", proportionEncoder);
                callingOpMode.telemetry.addData("pHeading", proportionHeading);
                callingOpMode.telemetry.addData("arc", proportionArc);
                callingOpMode.telemetry.addData("correction", correction);
                callingOpMode.telemetry.addData("leftPower", leftPower);
                callingOpMode.telemetry.addData("rightPower", rightPower);
                callingOpMode.telemetry.addData("loops", loops);
                callingOpMode.telemetry.update();
                break;
            }
        }
    }

    public void grabFoundation () throws InterruptedException {
        foundRight.setPosition(0);
        foundLeft.setPosition(0.75);
        Thread.sleep(1100);
    }

    public void releaseFoundation () throws InterruptedException {
        foundRight.setPosition(0.75);
        foundLeft.setPosition(0);
        Thread.sleep(1100);
    }

    public void mainFlopDown () throws InterruptedException {
        mainFlop.setPosition(0);
        Thread.sleep(400);
    }

    public void mainFlopMid () throws InterruptedException {
        mainFlop.setPosition(0.15);
        Thread.sleep(400);
    }

    public void mainFlopUp () throws InterruptedException {
        mainFlop.setPosition(0.7);
        subFlop.setPosition(0.35);
        Thread.sleep(400);
    }

    public void grabStone () throws InterruptedException {
        subFlop.setPosition(0.35);
        Thread.sleep(400);
    }

    public void releaseStone () throws InterruptedException {
        subFlop.setPosition(0.7);
        Thread.sleep(400);
    }



    /*
    *   pos 1 == closed & up
    *   pos 2 == open & mid
    *   pos 3 == closed & mid
    *   pos 4 == open & down
    *   pos 5 == closed & down
     */

    public void pos1 () throws InterruptedException {
        pos = 1;
        mainFlop.setPosition(0.7);
        subFlop.setPosition(0.35);
        Thread.sleep(400);
    }

    public void pos2 () throws InterruptedException {
        pos = 2;
        mainFlop.setPosition(0.15);
        subFlop.setPosition(0.7);
        Thread.sleep(400);
    }

    public void pos3 () throws InterruptedException {
        pos = 3;
        mainFlop.setPosition(0.15);
        subFlop.setPosition(0.35);
        Thread.sleep(400);
    }

    public void pos4 () throws InterruptedException {
        pos = 4;
        mainFlop.setPosition(0);
        subFlop.setPosition(0.7);
        Thread.sleep(400);
    }

    public void pos5 () throws InterruptedException {
        pos = 5;
        mainFlop.setPosition(0);
        subFlop.setPosition(0.35);
        Thread.sleep(400);
    }

    public int getPos () {
        return pos;
    }



    public void lift3F () throws InterruptedException {
        if (lift.getCurrentPosition() < 1000) {
            lift.setPower(0.9);
            while (lift.getCurrentPosition() < 1000) {Thread.sleep(10);}
        } else if (lift.getCurrentPosition() > 1000) {
            lift.setPower(-0.9);
            while (lift.getCurrentPosition() > 1000) {Thread.sleep(10);}
        }
        lift.setPower(0);
    }

    public void lift2F () throws InterruptedException {
        if (lift.getCurrentPosition() < 500) {
            lift.setPower(0.9);
            while (lift.getCurrentPosition() < 500) {Thread.sleep(10);}
        } else if (lift.getCurrentPosition() > 500) {
            lift.setPower(-0.9);
            while (lift.getCurrentPosition() > 500) {Thread.sleep(10);}
        }
        lift.setPower(0);
    }

    public void lift1F () throws InterruptedException {
        if (lift.getCurrentPosition() < 100) {
            lift.setPower(0.9);
            while (lift.getCurrentPosition() < 100) {Thread.sleep(10);}
        } else if (lift.getCurrentPosition() > 100) {
            lift.setPower(-0.9);
            while (lift.getCurrentPosition() > 100) {Thread.sleep(10);}
        }
        lift.setPower(0);
    }

    public void liftReset () throws InterruptedException {
        if (lift.getCurrentPosition() < 10) {
            lift.setPower(0.9);
            while (lift.getCurrentPosition() < 10) {Thread.sleep(10);}
        } else if (lift.getCurrentPosition() > 10) {
            lift.setPower(-0.9);
            while (lift.getCurrentPosition() > 10) {Thread.sleep(10);}
        }
        lift.setPower(0);
    }

    protected float normalize360two(float val) {
        while (val > 180f || val < -180f) {

            if (val > 180f) {
                val -= 360f;
            }

            if (val < -180f) {
                val += 360f;
            }
        }
        return val;
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