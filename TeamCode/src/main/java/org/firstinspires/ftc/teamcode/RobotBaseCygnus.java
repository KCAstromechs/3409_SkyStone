package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static android.content.Context.SENSOR_SERVICE;

@SuppressWarnings({"WeakerAccess", "FieldCanBeLocal"})
public class RobotBaseCygnus implements SensorEventListener {

    DcMotor frontRight, frontLeft, backRight, backLeft, encoderWheelY, encoderWheelX, liftL, liftR;
    Servo grab;
    DistanceSensor distSensor;

    OpMode callingOpMode;

    private double ticksPerRotation = 488;
    private double ticksPerRotationTetrix = 1400;

    private double wheelDiameter = 4;

    double ticksPerInch = ((ticksPerRotation)/(wheelDiameter * Math.PI));
    double ticksPerInchTetrix = ((ticksPerRotationTetrix)/(3 * Math.PI));

    protected static final double P_DRIVE_COEFF = 0.021;

    double L_P_COEFF = 0.008;
    int liftTarget = 0;

    static final double driveSpeed = 0.9;

    int frontRightBaseEncoder, frontLeftBaseEncoder, backRightBaseEncoder, encoderWheelYBaseEncoder, encoderWheelXBaseEncoder, backLeftBaseEncoder = 0;


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

    double global_y1, global_y2, global_x1, global_x2;
    double global_theta;
    double global_posF, global_posS;
    double globalX;
    double globalY;
    double global_deltaPosForwards = 0;
    double global_deltaPosSideways = 0;
    double global_lastForwardPos = 0;
    double global_lastSidewaysPos = 0;

    public RobotBaseCygnus(OpMode _callingOpMode) {
        callingOpMode = _callingOpMode;

        frontRight = callingOpMode.hardwareMap.dcMotor.get("frontRight");
        frontLeft = callingOpMode.hardwareMap.dcMotor.get("frontLeft");
        backRight = callingOpMode.hardwareMap.dcMotor.get("backRight");
        backLeft = callingOpMode.hardwareMap.dcMotor.get("backLeft");
        encoderWheelY = callingOpMode.hardwareMap.dcMotor.get("encoderWheelY");
        encoderWheelX = callingOpMode.hardwareMap.dcMotor.get("encoderWheelX");
        liftL = callingOpMode.hardwareMap.dcMotor.get("liftL");
        liftR = callingOpMode.hardwareMap.dcMotor.get("liftR");

        grab = callingOpMode.hardwareMap.servo.get("grab");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        liftL.setDirection(DcMotor.Direction.REVERSE);
        encoderWheelY.setDirection(DcMotor.Direction.REVERSE);
        encoderWheelX.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderWheelY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderWheelX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderWheelY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderWheelX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mSensorManager = (SensorManager) _callingOpMode.hardwareMap.appContext.getSystemService(SENSOR_SERVICE);
        mRotationVectorSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR);
        mSensorManager.registerListener(this, mRotationVectorSensor, 10000);
        hasBeenZeroed = false;
        globalY = 0;
        globalX = 0;
        grabUp();
        liftR.setPower(-0.15);
        liftL.setPower(-0.15);
        ((LinearOpMode) callingOpMode).sleep(1000);
        liftR.setPower(0);
        liftL.setPower(0);
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
        encoderWheelY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderWheelX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderWheelY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderWheelX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Thread.sleep(10);
    }

    public void stop() throws InterruptedException {

        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        Thread.sleep(200);
    }

    public void setEncoderBase(){
        frontRightBaseEncoder = frontRight.getCurrentPosition();
        frontLeftBaseEncoder = frontLeft.getCurrentPosition();
        backRightBaseEncoder = backRight.getCurrentPosition();
        backLeftBaseEncoder = backLeft.getCurrentPosition();
        encoderWheelYBaseEncoder = encoderWheelY.getCurrentPosition();
        encoderWheelXBaseEncoder = encoderWheelX.getCurrentPosition();
    }

    public void updateGlobalPosition(){
        //global coordinates calculation
        global_posF = encoderWheelY.getCurrentPosition();
        global_posS = encoderWheelX.getCurrentPosition();

        global_deltaPosForwards = (global_posF - global_lastForwardPos);
        global_deltaPosSideways = (global_posS - global_lastSidewaysPos);

        global_lastForwardPos = global_posF;
        global_lastSidewaysPos = global_posS;

        global_theta = zRotation * Math.PI / 180.0;
        global_y1 = global_deltaPosForwards * Math.cos(global_theta);
        global_y2 = -global_deltaPosSideways * Math.sin(global_theta);
        global_x1 = global_deltaPosForwards * Math.sin(global_theta);
        global_x2 = global_deltaPosSideways * Math.cos(global_theta);

        // these represent the current x,y in global coordinates ... in inches
        globalY += ticksToInches(global_y1 + global_y2);
        globalX += ticksToInches(global_x1 + global_x2);
    }

    public void lifterHandler(){
        double distanceL = liftTarget-getCurrentLiftPosition();
        double liftPower = (distanceL * L_P_COEFF);
        liftL.setPower(liftPower);
        liftR.setPower(liftPower);
    }

    public void setLiftTarget(int floor, int modifier){
        liftTarget = (280*floor)+modifier;
    }

    public int getCurrentFrontRightPosition() {return Math.abs(frontRight.getCurrentPosition()-frontRightBaseEncoder);}

    public int getCurrentFrontLeftPosition() {return Math.abs(frontLeft.getCurrentPosition()-frontLeftBaseEncoder);}

    public int getCurrentBackRightPosition() {return Math.abs(backRight.getCurrentPosition()-backRightBaseEncoder);}

    public int getCurrentBackLeftPosition() {return Math.abs(backLeft.getCurrentPosition()-backLeftBaseEncoder);}

    public int getCurrentLiftPosition() {return Math.abs((liftL.getCurrentPosition()+liftR.getCurrentPosition())/2);} //280 ticks per floor

    public int getEncoderWheelYPosition() {return encoderWheelY.getCurrentPosition()-encoderWheelYBaseEncoder;}

    public int getEncoderWheelXPosition() {return encoderWheelX.getCurrentPosition()-encoderWheelXBaseEncoder;}

    //public double getDistSensorInch() {return distSensor.getDistance(DistanceUnit.INCH);}

    public double ticksToInches(double x) { return 3 * Math.PI * x/1400; }

    public void driveStraight(double deltaY, double deltaX, float heading, double speedLimit)  throws InterruptedException {
        double error;                                           //The number of degrees between the true heading and desired heading
        double correction;                                      //Modifies power to account for error
        double max;
        double Y_P_COEFF = 0.17825353626292278;      //0.002
        double Y_I_COEFF = 0.2852056580206765;     //0.0035
        double Y_D_COEFF = 0.0003;    //0.00042
        double X_P_COEFF = 0.17825353626292278;      //0.002
        double X_I_COEFF = 0.2852056580206765;     //0.0035
        double X_D_COEFF = 0.0003;    //0.00042
        double driveY, driveX;
        double distanceY;
        double distanceX;
        long loops = 0;
        heading = (int) normalize360(heading);

        double frontLeft;
        double frontRight;
        double backLeft;
        double backRight;

        setEncoderBase();
        int lastEncoderY = 0;
        int lastEncoderX = 0;

        speedLimit = Range.clip(speedLimit, 0.0, 1.0);

        if (speedLimit==0){
            return;
        }

        double power;
        double deltaT;
        double derivativeY = 0;
        double derivativeX = 0;
        double integralY = 0;
        double integralX = 0;
        int deltaDY;
        int deltaDX;

        double y1, y2, x1, x2;
        double theta;
        double theta2;
        double gamma;
        double h;
        double posF, posS;
        double local_globalX = 0;
        double local_globalY = 0;
        double deltaPosForwards = 0;
        double deltaPosSideways = 0;
        double lastForwardPos = 0;
        double lastSidewaysPos = 0;

        double lastTime = callingOpMode.getRuntime();

        while (((((Math.abs(deltaY)-0.5) > Math.abs(local_globalY)) || ((Math.abs(deltaY)+0.5) < Math.abs(local_globalY)))  ||
                (((Math.abs(deltaX)-0.5) > Math.abs(local_globalX)) || ((Math.abs(deltaX)+0.5) < Math.abs(local_globalX)))) &&
                ((LinearOpMode) callingOpMode).opModeIsActive()) {

            error = heading - zRotation;

            while (error > 180) error = (error - 360);
            while (error <= -180) error = (error + 360);

            correction = Range.clip(error * P_DRIVE_COEFF, -1, 1);


            posF = getEncoderWheelYPosition();
            posS = getEncoderWheelXPosition();

            deltaPosForwards = (posF - lastForwardPos);
            deltaPosSideways = (posS - lastSidewaysPos);

            lastForwardPos = posF;
            lastSidewaysPos = posS;

            theta = zRotation * Math.PI / 180.0;
            y1 = deltaPosForwards * Math.cos(theta);
            y2 = -deltaPosSideways * Math.sin(theta);
            x1 = deltaPosForwards * Math.sin(theta);
            x2 = deltaPosSideways * Math.cos(theta);

            // these represent the current x,y in global coordinates ... in inches
            local_globalY += ticksToInches(y1 + y2);
            local_globalX += ticksToInches(x1 + x2);

             // these represent the current x and y translations still to do, in global coordinates ... in inches
            driveY = deltaY - local_globalY;
            driveX = deltaX - local_globalX;

            theta2 = Math.atan2(driveY, driveX);
            gamma = theta+theta2;
            h = Math.sqrt((driveX*driveX)+(driveY*driveY));

            // in robot coordinates,
            distanceY = Math.sin(gamma)*h;
            distanceX = Math.cos(gamma)*h;

            deltaT = callingOpMode.getRuntime()-lastTime;
            lastTime = callingOpMode.getRuntime();
            deltaDY = getEncoderWheelYPosition()-lastEncoderY;
            lastEncoderY = getEncoderWheelYPosition();
            deltaDX = getEncoderWheelXPosition()-lastEncoderX;
            lastEncoderX = getEncoderWheelXPosition();

            derivativeY = ((double) deltaDY)/deltaT;
            derivativeX = ((double) deltaDX)/deltaT;

            if(Math.abs(distanceY*Y_P_COEFF)<1){
                integralY += distanceY*deltaT;
            } else {
                integralY = 0;
            }
            if(Math.abs(distanceX*X_P_COEFF)<1){
                integralX += distanceX*deltaT;
            } else {
                integralX = 0;
            }

            power = (distanceY*Y_P_COEFF) + (integralY*Y_I_COEFF) - (derivativeY*Y_D_COEFF);

            frontLeft = power;
            frontRight = power;
            backLeft = power;
            backRight = power;

            power = (distanceX*X_P_COEFF) + (integralX*X_I_COEFF) - (derivativeX*X_D_COEFF);

            frontLeft += power;
            frontRight -= power;
            backLeft -= power;
            backRight += power;

            max = Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)), Math.max(Math.abs(backLeft), Math.abs(backRight)));
            if (max > Math.abs(speedLimit)) {
                frontLeft /= max;
                frontRight /= max;
                backLeft /= max;
                backRight /= max;
                frontLeft *= Math.abs(speedLimit);
                frontRight *= Math.abs(speedLimit);
                backLeft *= Math.abs(speedLimit);
                backRight *= Math.abs(speedLimit);
            }

            frontLeft += correction;
            frontRight -= correction;
            backLeft += correction;
            backRight -= correction;

            max = Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)), Math.max(Math.abs(backLeft), Math.abs(backRight)));
            if (max > 1.0) {
                frontLeft /= max;
                frontRight /= max;
                backLeft /= max;
                backRight /= max;
            }

            updateDriveMotors(frontLeft, frontRight, backLeft, backRight);

            if (((loops+10) % 10) ==  0) {
                callingOpMode.telemetry.addData("zRotation" , zRotation);
                callingOpMode.telemetry.addData("globalY" , local_globalY);
                callingOpMode.telemetry.addData("globalX" , local_globalX);
                callingOpMode.telemetry.addData("distanceY", distanceY);
                callingOpMode.telemetry.addData("distanceX", distanceX);
                callingOpMode.telemetry.addData("theta", theta);
                callingOpMode.telemetry.addData("theta2",theta2);
                callingOpMode.telemetry.addData("gamma",gamma);
                callingOpMode.telemetry.addData("driveY",driveY);
                callingOpMode.telemetry.addData("driveX",driveX);
                callingOpMode.telemetry.addData("loops", loops);
                callingOpMode.telemetry.update();
            }

            loops++;

            updateGlobalPosition();
            lifterHandler();

            ((LinearOpMode) callingOpMode).sleep(10);
        }
    }

    /**
     * drives to a global coordinate expressed as (x, y, theta)
     * @param targetX target global x coordinate
     * @param targetY target global y coordinate
     * @param heading target global angle (theta)
     * @param speedLimit speed limit on the PID loop (0.0 < speedLimit <= 1.0)
     * @throws InterruptedException
     */
    public void driveTo(double targetX, double targetY, float heading, double speedLimit)  throws InterruptedException {
        float error = heading - zRotation;                      //The number of degrees between the true heading and desired heading
        double correction;                                      //Modifies power to account for error
        double max;
        double Y_P_COEFF = 0.17825353626292278;
        double Y_I_COEFF = 0.2852056580206765;
        double Y_D_COEFF = 0.0003;
        double X_P_COEFF = 0.225;
        double X_I_COEFF = 0.37869858549483094;
        double X_D_COEFF = 0.00045;
        double T_P_COEFF = 0.0168;
        double T_I_COEFF = 0.0240;
        double T_D_COEFF = 0.00296;
        double driveY, driveX;
        double distanceY;
        double distanceX;
        long loops = 0;
        heading = (int) normalize360(heading);

        double frontLeft;
        double frontRight;
        double backLeft;
        double backRight;

        setEncoderBase();
        int lastEncoderY = 0;
        int lastEncoderX = 0;
        float lastHeading = zRotation;
        float lastError = error;

        speedLimit = Range.clip(speedLimit, 0.0, 1.0);

        if (speedLimit==0){
            return;
        }

        double power;
        double deltaT;
        double derivativeY = 0;
        double derivativeX = 0;
        double derivativeT = 0;
        double integralY = 0;
        double integralX = 0;
        double integralT = 0;
        int deltaDY;
        int deltaDX;
        float deltaDT;

        double theta;
        double theta2;
        double gamma;
        double h;
        double local_globalX;
        double local_globalY;

        double lastTime = callingOpMode.getRuntime();

        while ((((((Math.abs(targetY)-1) > Math.abs(globalY)) || ((Math.abs(targetY)+1) < Math.abs(globalY)))  ||
                (((Math.abs(targetX)-1) > Math.abs(globalX)) || ((Math.abs(targetX)+1) < Math.abs(globalX))))  ||
                (Math.abs(error)>=5.0f)) &&  ((LinearOpMode) callingOpMode).opModeIsActive()) {

            lastError = error;
            error = heading - zRotation;

            while (error > 180) error = (error - 360);
            while (error <= -180) error = (error + 360);

            local_globalX = globalX;
            local_globalY = globalY;

            theta = zRotation * Math.PI / 180.0;

            // these represent the current x and y translations still to do, in global coordinates ... in inches
            driveY = targetY - local_globalY;
            driveX = targetX - local_globalX;

            theta2 = Math.atan2(driveY, driveX);
            gamma = theta+theta2;
            h = Math.sqrt((driveX*driveX)+(driveY*driveY));

            // in robot coordinates,
            distanceY = Math.sin(gamma)*h;
            distanceX = Math.cos(gamma)*h;

            deltaT = callingOpMode.getRuntime()-lastTime;
            lastTime = callingOpMode.getRuntime();
            deltaDY = getEncoderWheelYPosition()-lastEncoderY;
            lastEncoderY = getEncoderWheelYPosition();
            deltaDX = getEncoderWheelXPosition()-lastEncoderX;
            lastEncoderX = getEncoderWheelXPosition();
            deltaDT = zRotation-lastHeading;
            lastHeading = zRotation;

            derivativeY = ((double) deltaDY)/deltaT;
            derivativeX = ((double) deltaDX)/deltaT;
            derivativeT = ((double) deltaDT/deltaT);

            if(Math.abs(distanceY*Y_P_COEFF)<1 && Math.abs(distanceY)>1){
                integralY += distanceY*deltaT;
            } else {
                integralY = 0;
            }
            if(Math.abs(distanceX*X_P_COEFF)<1 && Math.abs(distanceX)>1){
                integralX += distanceX*deltaT;
            } else {
                integralX = 0;
            }
            if(Math.abs(error*T_P_COEFF)>1 || (Math.abs(error)<5 && Math.abs(lastError)>5)){
                integralT = 0;
            } else {
                integralT += error*deltaT;
            }

            power = (distanceY*Y_P_COEFF) + (integralY*Y_I_COEFF) - (derivativeY*Y_D_COEFF);

            frontLeft = power;
            frontRight = power;
            backLeft = power;
            backRight = power;

            power = (distanceX*X_P_COEFF) + (integralX*X_I_COEFF) - (derivativeX*X_D_COEFF);

            frontLeft += power;
            frontRight -= power;
            backLeft -= power;
            backRight += power;

            max = Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)), Math.max(Math.abs(backLeft), Math.abs(backRight)));
            if (max > Math.abs(speedLimit)) {
                frontLeft /= max;
                frontRight /= max;
                backLeft /= max;
                backRight /= max;
                frontLeft *= Math.abs(speedLimit);
                frontRight *= Math.abs(speedLimit);
                backLeft *= Math.abs(speedLimit);
                backRight *= Math.abs(speedLimit);
            }

            correction = (error*T_P_COEFF) + (integralT*T_I_COEFF) - (derivativeT*T_D_COEFF);

            frontLeft += correction;
            frontRight -= correction;
            backLeft += correction;
            backRight -= correction;

            max = Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)), Math.max(Math.abs(backLeft), Math.abs(backRight)));
            if (max > 1.0) {
                frontLeft /= max;
                frontRight /= max;
                backLeft /= max;
                backRight /= max;
            }

            updateDriveMotors(frontLeft, frontRight, backLeft, backRight);

            if (((loops+10) % 10) ==  0) {
                callingOpMode.telemetry.addData("zRotation" , zRotation);
                callingOpMode.telemetry.addData("globalY" , local_globalY);
                callingOpMode.telemetry.addData("globalX" , local_globalX);
                callingOpMode.telemetry.addData("distanceY", distanceY);
                callingOpMode.telemetry.addData("distanceX", distanceX);
                callingOpMode.telemetry.addData("theta", theta);
                callingOpMode.telemetry.addData("theta2",theta2);
                callingOpMode.telemetry.addData("gamma",gamma);
                callingOpMode.telemetry.addData("driveY",driveY);
                callingOpMode.telemetry.addData("driveX",driveX);
                callingOpMode.telemetry.addData("error", error);
                callingOpMode.telemetry.addData("loops", loops);
                callingOpMode.telemetry.update();
            }

            loops++;

            updateGlobalPosition();
            lifterHandler();

            ((LinearOpMode) callingOpMode).sleep(10);
        }
    }

    public void turn(float heading, double speedLimit){
        float error = heading - zRotation;                      //The number of degrees between the true heading and desired heading
        double correction;                                      //Modifies power to account for error
        double max;
        double T_P_COEFF = 0.0168;
        double T_I_COEFF = 0.0240;
        double T_D_COEFF = 0.00296;
        long loops = 0;
        heading = (int) normalize360(heading);

        double frontLeft;
        double frontRight;
        double backLeft;
        double backRight;

        float lastHeading = zRotation;

        speedLimit = Range.clip(speedLimit, 0.0, 1.0);

        if (speedLimit==0){
            return;
        }
        double deltaT;
        double derivativeT = 0;
        double integralT = 0;
        float deltaDT;

        double lastTime = callingOpMode.getRuntime();

        while ((Math.abs(error)>=5.0f) &&  ((LinearOpMode) callingOpMode).opModeIsActive()) {
            error = heading - zRotation;

            while (error > 180) error = (error - 360);
            while (error <= -180) error = (error + 360);

            deltaT = callingOpMode.getRuntime()-lastTime;
            lastTime = callingOpMode.getRuntime();

            deltaDT = zRotation-lastHeading;
            lastHeading = zRotation;

            derivativeT = ((double) deltaDT/deltaT);

            if(Math.abs(error*T_P_COEFF)<1){
                integralT += error*deltaT;
            } else {
                integralT = 0;
            }

            correction = (error*T_P_COEFF) + (integralT*T_I_COEFF) - (derivativeT*T_D_COEFF);

            frontLeft = correction;
            frontRight = -correction;
            backLeft = correction;
            backRight = -correction;

            max = Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)), Math.max(Math.abs(backLeft), Math.abs(backRight)));
            if (max > 1.0) {
                frontLeft /= max;
                frontRight /= max;
                backLeft /= max;
                backRight /= max;
            }

            updateDriveMotors(frontLeft, frontRight, backLeft, backRight);

            if (((loops+10) % 10) ==  0) {
                callingOpMode.telemetry.addData("zRotation" , zRotation);
                callingOpMode.telemetry.addData("heading", heading);
                callingOpMode.telemetry.addData("error", error);
                callingOpMode.telemetry.addData("correction", correction);
                callingOpMode.telemetry.addData("loops", loops);
                callingOpMode.telemetry.update();
            }

            loops++;

            updateGlobalPosition();
            lifterHandler();

            ((LinearOpMode) callingOpMode).sleep(10);
        }
    }

    /*public void turn(float turnHeading, double speedLimit) throws InterruptedException {
        int wrapFix = 0;                                        //Can be used to modify values and make math around 0 easier
        float shiftedTurnHeading = turnHeading;                 //Can be used in conjunction with wrapFix to make math around 0 easier
        long loops = 0;

        speedLimit = Math.abs(speedLimit);                                //makes sure the speedLimit is positive
        if (speedLimit>1) speedLimit = 1;                                 //makes sure the speedLimit isn't >1

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

            updateDriveMotors(-speedLimit, speedLimit, -speedLimit, speedLimit);

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

                lifterHandler();
                updateGlobalPosition();

                //Chill a hot decisecond
                Thread.sleep(10);
            }
        }
        //If it would take longer to take the cwise path, we go *** COUNTERCLOCKWISE ***
        else if(Math.abs(clockwise) > Math.abs(cclockwise)) {

            updateDriveMotors(speedLimit, -speedLimit, speedLimit, -speedLimit);

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

                lifterHandler();
                updateGlobalPosition();

                //Hold up a hot decisecond
                Thread.sleep(10);
            }
        }
    }*/


    public void grabUp(){
        grab.setPosition(0);
    }

    public void grabOpen() {
        grab.setPosition(0.5);
    }

    public void grabClose() {
        grab.setPosition(0.76);
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

    public void deconstruct(){
        mSensorManager.unregisterListener(this);
    }
}