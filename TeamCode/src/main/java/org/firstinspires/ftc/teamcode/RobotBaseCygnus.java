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
    Servo grab, cloyer;
    DistanceSensor distR, distL;

    OpMode callingOpMode;

    private double ticksPerRotation = 488;
    private double ticksPerRotationTetrix = 1400;

    private double wheelDiameter = 4;

    double ticksPerInch = ((ticksPerRotation)/(wheelDiameter * Math.PI));
    double ticksPerInchTetrix = ((ticksPerRotationTetrix)/(3 * Math.PI));

    protected static final double P_DRIVE_COEFF = 0.021;

    double L_P_COEFF = 0.006;
    int liftTarget = 0;
    double liftTimer = 0;

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
        cloyer = callingOpMode.hardwareMap.servo.get("cloyer");

        distL = callingOpMode.hardwareMap.get(DistanceSensor.class, "distL");
        distR = callingOpMode.hardwareMap.get(DistanceSensor.class, "distR");

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
        if(mRotationVectorSensor==null){
            System.out.println("nope");
            callingOpMode.telemetry.addData("nope", "no");
            callingOpMode.telemetry.update();
        } else {
            System.out.println("yep");
            callingOpMode.telemetry.addData("yep", "yes");
            callingOpMode.telemetry.update();
        }
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


        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        cloyer.setPosition(0);
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

        /*global_posS += Math.tan(Math.toRadians(5))*global_posF;
        global_posS *= Math.cos(Math.toRadians(5));*/

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

    public void setGlobalY (double y) {
        globalY = y;
    }

    public void setGlobalX (double x) {
        globalX = x;
    }

    double liftSpeedLimit = 0.6;

    public void lifterHandler(){
        double liftPower;
        if(callingOpMode.getRuntime()>liftTimer&&liftTimer!=0){
            liftPower=0;
        } else {
            double distanceL = liftTarget-getCurrentLiftPosition();
            liftPower = (distanceL * L_P_COEFF);
        }
        if(liftPower<-liftSpeedLimit) liftPower = -liftSpeedLimit;
        liftL.setPower(liftPower);
        liftR.setPower(liftPower);
    }

    public double dropTimer = 1;

    public void dropLifter(){
        liftTarget = 0;
        liftTimer = callingOpMode.getRuntime()+dropTimer;
    }

    public void setLiftTarget(int floor, int modifier){
        liftTarget = (280*floor)+modifier;
        liftTimer = 0;
    }

    public int getCurrentFrontRightPosition() {return Math.abs(frontRight.getCurrentPosition()-frontRightBaseEncoder);}

    public int getCurrentFrontLeftPosition() {return Math.abs(frontLeft.getCurrentPosition()-frontLeftBaseEncoder);}

    public int getCurrentBackRightPosition() {return Math.abs(backRight.getCurrentPosition()-backRightBaseEncoder);}

    public int getCurrentBackLeftPosition() {return Math.abs(backLeft.getCurrentPosition()-backLeftBaseEncoder);}

    public int getCurrentLiftPosition() {return liftL.getCurrentPosition();} //280 ticks per floor

    public int getEncoderWheelYPosition() {return encoderWheelY.getCurrentPosition()-encoderWheelYBaseEncoder;}

    public int getEncoderWheelXPosition() {return encoderWheelX.getCurrentPosition()-encoderWheelXBaseEncoder;}

    public double getDistRInch() throws IllegalDistanceException {return getDistRInch(30.0);}

    public double getDistRInch(double max) throws IllegalDistanceException {
        double _distR = distR.getDistance(DistanceUnit.INCH);
        int loops = 1;
        while((_distR==0.0 || _distR>max)&&((LinearOpMode) callingOpMode).opModeIsActive()) {
            _distR = distR.getDistance(DistanceUnit.INCH);
            callingOpMode.telemetry.addData("distR", _distR);
            callingOpMode.telemetry.update();
            ((LinearOpMode) callingOpMode).sleep(50);
            updateGlobalPosition();
            lifterHandler();
            loops++;
            if(loops>=6) break;
        }
        if(loops>=6) throw new IllegalDistanceException(String.valueOf(_distR));
        return _distR;
    }

    public double getDistRInchInit(double max) {
        double _distR = distR.getDistance(DistanceUnit.INCH);
        while((_distR==0.0 || _distR>max)) {
            _distR = distR.getDistance(DistanceUnit.INCH);
            callingOpMode.telemetry.addData("distR", _distR);
            callingOpMode.telemetry.update();
            ((LinearOpMode) callingOpMode).sleep(50);
        }
        return _distR;
    }

    public double getDistLInch() throws IllegalDistanceException {return getDistLInch(30.0);}

    public double getDistLInch(double max) throws IllegalDistanceException {
        double _distL = distL.getDistance(DistanceUnit.INCH);
        int loops = 1;
        while((_distL==0.0 || _distL>max)&&((LinearOpMode) callingOpMode).opModeIsActive()) {
            _distL = distL.getDistance(DistanceUnit.INCH);
            callingOpMode.telemetry.addData("distL", _distL);
            callingOpMode.telemetry.update();
            ((LinearOpMode) callingOpMode).sleep(50);
            updateGlobalPosition();
            lifterHandler();
            loops++;
            if(loops>=6) break;
        }
        if(loops>=6) throw new IllegalDistanceException(String.valueOf(_distL));
        return _distL;
    }

    public double getDistLInchInit(double max) {
        double _distL = distL.getDistance(DistanceUnit.INCH);
        while((_distL==0.0 || _distL>max)) {
            _distL = distL.getDistance(DistanceUnit.INCH);
            callingOpMode.telemetry.addData("distL", _distL);
            callingOpMode.telemetry.update();
            ((LinearOpMode) callingOpMode).sleep(50);
        }
        return _distL;
    }

    public double getDistAverage() throws IllegalDistanceException {return getDistAverage(30.0);}

    public double getDistAverage(double max)  throws IllegalDistanceException {
        double _distL;
        double _distR;
        boolean l = false;
        boolean r = false;
        try {
            _distL = getDistLInch(max);
            l = true;
        } catch (IllegalDistanceException e) {
            _distL = 0;
        }
        try {
            _distR = getDistRInch(max);
            r = true;
        } catch (IllegalDistanceException e) {
            _distR = 0;
        }
        if(l){
            if(r){
                return ((_distL + _distR) / 2);
            } else {
                return _distL;
            }
        } else if(r){
            return _distR;
        } else {
            throw new IllegalDistanceException(String.valueOf(((_distL + _distR) / 2)));
        }
    }

    public double ticksToInches(double x) { return 3 * Math.PI * x/1400; }

    public void driveStraight(double deltaY, double deltaX, float heading, double speedLimit)  throws InterruptedException {
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

        while ((((((Math.abs(deltaY)-1) > Math.abs(local_globalY)) || ((Math.abs(deltaY)+1) < Math.abs(local_globalY)))  ||
                (((Math.abs(deltaX)-1) > Math.abs(local_globalX)) || ((Math.abs(deltaX)+1) < Math.abs(local_globalX))))  ||
                (Math.abs(error)>=5.0f)) &&  ((LinearOpMode) callingOpMode).opModeIsActive()) {

            error = heading - zRotation;

            while (error > 180) error = (error - 360);
            while (error <= -180) error = (error + 360);

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

    public double intermediateRange = 4;

    /**
     * drives to a global coordinate expressed as (x, y, theta)
     * @param targetX target global x coordinate
     * @param targetY target global y coordinate
     * @param heading target global angle (theta)
     * @param speedLimit speed limit on the PID loop (0.0 < speedLimit <= 1.0)
     * sets timeout to 0; never abort based on time
     * @throws InterruptedException
     */
    public void driveTo(double targetX, double targetY, float heading, double speedLimit)  throws InterruptedException {
        driveTo(targetX, targetY, heading, speedLimit, 0);
    }

    /**
     * drives to a global coordinate expressed as (x, y, theta)
     * @param targetX target global x coordinate
     * @param targetY target global y coordinate
     * @param heading target global angle (theta)
     * @param speedLimit speed limit on the PID loop (0.0 < speedLimit <= 1.0)
     * @param timeout time, in seconds, after which the drive aborts (0.0 <= timeout)
     * if timeout == 0, never abort based on time
     * @throws InterruptedException
     */
    public void driveTo(double targetX, double targetY, float heading, double speedLimit, double timeout)  throws InterruptedException {
        driveTo(targetX, targetY, heading, speedLimit, timeout, false);
    }

    /**
     * drives to a global coordinate expressed as (x, y, theta)
     * @param targetX target global x coordinate
     * @param targetY target global y coordinate
     * @param heading target global angle (theta)
     * @param speedLimit speed limit on the PID loop (0.0 < speedLimit <= 1.0)
     * sets timeout to 0; never abort based on time
     * @param intermediate sets driveTo to intermediate mode
     * @throws InterruptedException
     */
    public void driveTo(double targetX, double targetY, float heading, double speedLimit, boolean intermediate)  throws InterruptedException {
        driveTo(targetX, targetY, heading, speedLimit, 0, intermediate);
    }

    /**
     * drives to a global coordinate expressed as (x, y, theta)
     * @param targetX target global x coordinate
     * @param targetY target global y coordinate
     * @param heading target global angle (theta)
     * @param speedLimit speed limit on the PID loop (0.0 < speedLimit <= 1.0)
     * @param timeout time, in seconds, after which the drive aborts (0.0 <= timeout)
     * if timeout == 0, never abort based on time
     * @param intermediate sets driveTo to intermediate mode
     * @throws InterruptedException
     */
    public void driveTo(double targetX, double targetY, float heading, double speedLimit, double timeout, boolean intermediate)  throws InterruptedException {
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

        double acceptableX;
        double acceptableY;
        float acceptableT;

        if(!intermediate) {    //normal mode
            acceptableX = 1;
            acceptableY = 1;
            acceptableT = 7.5f;
        } else {               //intermediate mode
            acceptableX = intermediateRange;
            acceptableY = intermediateRange;
            acceptableT = 10f;
        }

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
        double startTime = lastTime;
        if(timeout<0){
            timeout=0;
        }

        while ((((((Math.abs(targetY)-acceptableY) > Math.abs(globalY)) || ((Math.abs(targetY)+acceptableY) < Math.abs(globalY)))  ||
                (((Math.abs(targetX)-acceptableX) > Math.abs(globalX)) || ((Math.abs(targetX)+acceptableX) < Math.abs(globalX))))  ||
                (Math.abs(error)>=acceptableT)) && (timeout==0 || lastTime<=startTime+timeout) &&
                ((LinearOpMode) callingOpMode).opModeIsActive()) {

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

            while (deltaDT > 180) deltaDT= (deltaDT - 360);
            while (deltaDT <= -180) deltaDT = (deltaDT + 360);

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
        if(!((LinearOpMode) callingOpMode).opModeIsActive()){
            updateDriveMotors(0, 0, 0, 0);
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

    public void pullFoundation (boolean right, float heading, double power){
        if(right){
            updateDriveMotors(0, -power, 0, -power);
        } else {
            updateDriveMotors(-power, 0, -power, 0);
        }
        float error = heading - zRotation;
        while((Math.abs(error)>=5.0f) &&  ((LinearOpMode) callingOpMode).opModeIsActive()){
            error = heading - zRotation;

            while (error > 180) error = (error - 360);
            while (error <= -180) error = (error + 360);

            updateGlobalPosition();
            lifterHandler();

            ((LinearOpMode) callingOpMode).sleep(10);
        }
        updateDriveMotors(0, 0, 0, 0);
        ((LinearOpMode) callingOpMode).sleep(200);
    }

    //method for turning foundation in one complete package
    public void turnFoundation (boolean right, float headingCutoff, double power){
        if(right){
            updateDriveMotors(0, -power, 0, -power);
        } else {
            updateDriveMotors(-power, 0, -power, 0);
        }
        float error = headingCutoff - zRotation;
        while((Math.abs(error)>=5.0f) &&  ((LinearOpMode) callingOpMode).opModeIsActive()){
            error = headingCutoff - zRotation;

            while (error > 180) error = (error - 360);
            while (error <= -180) error = (error + 360);

            updateGlobalPosition();
            lifterHandler();

            ((LinearOpMode) callingOpMode).sleep(10);
        }
        if(right){
            updateDriveMotors(power, -power, power, -power);
            headingCutoff = 85;
        } else {
            updateDriveMotors(-power, power, -power, power);
            headingCutoff = 275;
        }
        error = headingCutoff - zRotation;
        while((Math.abs(error)>=5.0f) &&  ((LinearOpMode) callingOpMode).opModeIsActive()){
            error = headingCutoff - zRotation;

            while (error > 180) error = (error - 360);
            while (error <= -180) error = (error + 360);

            updateGlobalPosition();
            lifterHandler();

            ((LinearOpMode) callingOpMode).sleep(10);
        }
    }

    public void pullTurn (boolean right, float heading, double power){
        if(right){
            updateDriveMotors(power, -power, power, -power);
        } else {
            updateDriveMotors(-power, power, -power, power);
        }
        float error = heading - zRotation;
        while((Math.abs(error)>=5.0f) &&  ((LinearOpMode) callingOpMode).opModeIsActive()){
            error = heading - zRotation;

            while (error > 180) error = (error - 360);
            while (error <= -180) error = (error + 360);

            updateGlobalPosition();
            lifterHandler();

            ((LinearOpMode) callingOpMode).sleep(10);
        }
    }

    public void pushFoundation (boolean right, float headingCutoff, double power){
        if(right){
            updateDriveMotors(power, -power, power, -power);
        } else {
            updateDriveMotors(-power, power, -power, power);
        }
        float error = headingCutoff - zRotation;
        while((Math.abs(error)>=5.0f) &&  ((LinearOpMode) callingOpMode).opModeIsActive()){
            error = headingCutoff - zRotation;

            while (error > 180) error = (error - 360);
            while (error <= -180) error = (error + 360);

            updateGlobalPosition();
            lifterHandler();

            ((LinearOpMode) callingOpMode).sleep(10);
        }
        if(right){
            updateDriveMotors(power, 0, power, 0);
            headingCutoff = 85;
        } else {
            updateDriveMotors(0, power, 0, power);
            headingCutoff = 275;
        }
        error = headingCutoff - zRotation;
        while((Math.abs(error)>=5.0f) &&  ((LinearOpMode) callingOpMode).opModeIsActive()){
            error = headingCutoff - zRotation;

            while (error > 180) error = (error - 360);
            while (error <= -180) error = (error + 360);

            updateGlobalPosition();
            lifterHandler();

            ((LinearOpMode) callingOpMode).sleep(10);
        }
    }


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
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);

        mSensorManager.unregisterListener(this);
    }




}