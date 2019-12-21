package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static android.content.Context.SENSOR_SERVICE;

@TeleOp(name="spike_GlobalCoordinatesTeleOp")
public class spike_GlobalCoordinatesTeleOp extends OpMode implements SensorEventListener {
    //init vars
    private float left, right, leftT, rightT, frontLeftPower, backLeftPower, frontRightPower, backRightPower;
    private DcMotor frontRight, frontLeft, backRight, backLeft, encoderWheel, encoderWheelHorizontal;
    private double error = 0;
    private int turbo = 9;
    private double globalX = 0;
    private double globalY = 0;
    private double deltaPosForwards = 0;
    private double deltaPosSideways = 0;
    private double deltaX = 0;
    private double deltaY = 0;
    private double loopsPerCalc = 0;
    private double lastPos = 0;
    private double currentPos = 0;
    private double lastForwardPos = 0;
    private double lastSidewaysPos = 0;
    private final double loopsPerAccumulate = 1;
    private int i;

    float zRotation;

    //arrays for gyro operation
    private float[] rotationMatrix = new float[9];
    private float[] orientation = new float[3];

    //objects for gyro operation
    private SensorManager mSensorManager;
    private Sensor mRotationVectorSensor;

    protected boolean hasBeenZeroed= false;

    //variables for gyro operation
    private float zero;
    private float rawGyro;
    public int sensorDataCounter = 0;

    private double spikeyOpen = 0;
    private double spikeyClosed = .4;

    private double grabbyOpen = 0.5;
    private double grabbyClosed = 1;

    double accumulationTimeStamp;

    @Override
    public void init() {
        mSensorManager = (SensorManager) hardwareMap.appContext.getSystemService(SENSOR_SERVICE);
        mRotationVectorSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR);
        mSensorManager.registerListener(this, mRotationVectorSensor, 10000);

        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        encoderWheel = hardwareMap.dcMotor.get("encoderWheelY");
        encoderWheelHorizontal = hardwareMap.dcMotor.get("encoderWheelX");

        encoderWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderWheelHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderWheelHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hasBeenZeroed = false;

        accumulationTimeStamp = System.currentTimeMillis();

        i = 0;
    }
    double y1, y2, x1, x2;
    double theta;
    double posF, posS;
    @Override
    public void loop() {
        turbo = 9;        //average displacement of drivetrain since last loop
        if(i % loopsPerAccumulate == 0) {
            telemetry.addData("ms per loop", System.currentTimeMillis() - accumulationTimeStamp);
            accumulationTimeStamp = System.currentTimeMillis();

            posF = encoderWheel.getCurrentPosition();
            posS = -encoderWheelHorizontal.getCurrentPosition();

            deltaPosForwards = (posF - lastForwardPos);
            deltaPosSideways = (posS - lastSidewaysPos);

            lastForwardPos = posF;
            lastSidewaysPos = posS;

            telemetry.addData("deltaPosForwards", deltaPosForwards);
            telemetry.addData("deltaPosSideways", deltaPosSideways);

            theta = zRotation * Math.PI / 180.0;
            y1 = deltaPosForwards * Math.cos(theta);
            y2 = -deltaPosSideways * Math.sin(theta);
            x1 = deltaPosForwards * Math.sin(theta);
            x2 = deltaPosSideways * Math.cos(theta);

            telemetry.addData("radians", theta);
            telemetry.addData("degrees", zRotation);

            globalY += ticksToInches(y1 + y2);
            globalX += ticksToInches(x1 + x2);

            telemetry.addData("globalY", globalY);
            telemetry.addData("globalX", globalX);

            telemetry.addData("forwardsPos", lastForwardPos);
            telemetry.addData("sidewaysPos", lastSidewaysPos);
        }
        i++;
        right = (Math.abs(gamepad1.left_stick_y) < 0.05) ? 0 : gamepad1.left_stick_y;
        left = (Math.abs(gamepad1.right_stick_y) < 0.05) ? 0 : gamepad1.right_stick_y;
        leftT = (Math.abs(gamepad1.left_trigger) < 0.05) ? 0 : gamepad1.left_trigger;
        rightT = (Math.abs(gamepad1.right_trigger) < 0.05) ? 0 : gamepad1.right_trigger;

        frontLeftPower = left - rightT + leftT;
        backLeftPower = left + rightT - leftT;
        frontRightPower = right + rightT - leftT;
        backRightPower = right - rightT + leftT;

        reducePowers(Math.max(frontLeftPower, Math.max(backLeftPower, Math.max(frontRightPower, backRightPower))));

        frontRight.setPower((frontRightPower*turbo)/10);
        backRight.setPower((backRightPower*turbo)/10);
        frontLeft.setPower((frontLeftPower*turbo)/10);
        backLeft.setPower((backLeftPower*turbo)/10);

        telemetry.update();
    }

    public double ticksToInches(double x) {
        return 3 * Math.PI * x/1400; //x/(1400/3*pi)
    }

    private void reducePowers(float power) {

        if (power > 1.0) {

            float multiplier = 1/power;

            frontLeftPower *= multiplier;
            frontRightPower *= multiplier;
            backLeftPower *= multiplier;
            backRightPower *= multiplier;
        }
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

    @Override
    public void stop() {}
}