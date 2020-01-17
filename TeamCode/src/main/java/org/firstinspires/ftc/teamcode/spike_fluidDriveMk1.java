package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static android.content.Context.SENSOR_SERVICE;

@Disabled
@Autonomous(name="spike_fluidDriveMk1")
public class spike_fluidDriveMk1 extends LinearOpMode implements SensorEventListener {

    private double ticksPerRotation = 488;

    private double wheelDiameter = 4;

    double ticksPerInch = ((ticksPerRotation)/(wheelDiameter * Math.PI));

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

    //init vars
    private DcMotor frontRight, frontLeft, backRight, backLeft;

    @Override
    public void runOpMode() throws InterruptedException {

        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mSensorManager = (SensorManager) hardwareMap.appContext.getSystemService(SENSOR_SERVICE);
        mRotationVectorSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR);
        mSensorManager.registerListener(this, mRotationVectorSensor, 10000);


        telemetry.addData("0", "1");
        telemetry.update();


        waitForStart();

        curveDrive(105.8, -90, 0.9, false);
        sleep(200);
        curveDrive(126.1, 90, -0.9, true);

        sleep(200000);
    }

    private void curveDrive (double inches, float degreesToTurn, double basePower, boolean clockwise) throws InterruptedException {

        //degrees to turn  positive for clockwise

        int target = (int) (inches * ticksPerInch);
        double correction;
        double leftPower = basePower;                                       //Power being fed to left side of bot
        double rightPower = basePower;                                      //Power being fed to right side of bot
        double max;                                             //To be used to keep powers from exceeding 1
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

        double P_ARC_COEFF = 3.8;

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sleep(10);

        frontRight.setPower(basePower);
        backRight.setPower(basePower);
        frontLeft.setPower(basePower);
        backLeft.setPower(basePower);

        sleep(10);

        while (opModeIsActive()) {

            lastEncoder = currentEncoder;
            currentEncoder = (backLeft.getCurrentPosition() + backRight.getCurrentPosition()) / 2;

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
                telemetry.addData("TARGET", target);
                telemetry.addData("gyro", zRotation);
                telemetry.addData("encoder", currentEncoder);
                telemetry.addData("heading", currentHeading);
                telemetry.addData("dEncoder", deltaEncoder);
                telemetry.addData("dHeading", deltaHeading);
                telemetry.addData("pEncoder", proportionEncoder);
                telemetry.addData("pHeading", proportionHeading);
                telemetry.addData("arc", proportionArc);
                telemetry.addData("correction", correction);
                telemetry.addData("leftPower", leftPower);
                telemetry.addData("rightPower", rightPower);
                telemetry.addData("loops", loops);
                telemetry.update();
            }

            loops++;

            sleep(10);

            if(Math.abs(target) <= Math.abs((backLeft.getCurrentPosition()+backRight.getCurrentPosition())/2)) {
                telemetry.addData("TARGET", target);
                telemetry.addData("gyro", zRotation);
                telemetry.addData("encoder", currentEncoder);
                telemetry.addData("heading", currentHeading);
                telemetry.addData("dEncoder", deltaEncoder);
                telemetry.addData("dHeading", deltaHeading);
                telemetry.addData("pEncoder", proportionEncoder);
                telemetry.addData("pHeading", proportionHeading);
                telemetry.addData("arc", proportionArc);
                telemetry.addData("correction", correction);
                telemetry.addData("leftPower", leftPower);
                telemetry.addData("rightPower", rightPower);
                telemetry.addData("loops", loops);
                telemetry.update();
                break;
            }
        }
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
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
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

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
}