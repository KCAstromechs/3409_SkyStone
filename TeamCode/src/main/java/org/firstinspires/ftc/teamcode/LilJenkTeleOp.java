package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="LilJenkTeleOp")
public class LilJenkTeleOp extends OpMode implements SensorEventListener {

    //init vars
    private float left, right, left2, right2, leftT, rightT, frontLeftPower, backLeftPower, frontRightPower, backRightPower;
    private DcMotor frontRight, frontLeft, backRight, backLeft, climbyBoi;
    private Servo spikeyBoi, grabbyBoi;
    private double error = 0;
    private int turbo = 9;
    private double globalX = 0;
    private double globalY = 0;
    private double deltaPos = 0;
    private double deltaX = 0;
    private double deltaY = 0;
    private double loopsPerCalc = 0;
    private double lastPos = 0;
    private double currentPos = 0;
    private double lastLeftPos = 0;
    private double lastRightPos = 0;
    private final double loopsPerAccumulate = 1;
    private int i = 0;

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



    @Override
    public void init() {
        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");

        climbyBoi = hardwareMap.dcMotor.get("climbyBoi");
        spikeyBoi = hardwareMap.servo.get("spikeyBoi");
        grabbyBoi = hardwareMap.servo.get("grabbyBoi");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hasBeenZeroed = false;

        spikeyBoi.setPosition(spikeyOpen);
        grabbyBoi.setPosition(grabbyOpen);
    }
    @Override
    public void loop() {
        turbo = 9;        //average displacement of drivetrain since last loop
        if(i % loopsPerAccumulate == 0) {
            deltaPos = (Math.abs(frontRight.getCurrentPosition() - lastRightPos) + Math.abs(frontLeft.getCurrentPosition() - lastLeftPos))/2;

            globalX += ticksToInches(deltaPos * Math.cos(zRotation));
            globalY += ticksToInches(deltaPos * Math.sin(zRotation));

            lastLeftPos = frontLeft.getCurrentPosition();
            lastRightPos = frontRight.getCurrentPosition();
        }
        i++;
        left = (Math.abs(gamepad1.left_stick_y) < 0.05) ? 0 : -1 * gamepad1.left_stick_y;
        right = (Math.abs(gamepad1.right_stick_y) < 0.05) ? 0 : -1 * gamepad1.right_stick_y;
        left2 = (Math.abs(gamepad2.left_stick_y) < 0.05) ? 0 : gamepad2.left_stick_y;
        right2 = (Math.abs(gamepad2.right_stick_y) < 0.1) ? 0 : gamepad2.right_stick_y;
        leftT = (Math.abs(gamepad1.left_trigger) < 0.05) ? 0 : gamepad1.left_trigger;
        rightT = (Math.abs(gamepad1.right_trigger) < 0.05) ? 0 : gamepad1.right_trigger;

        frontLeftPower = left - rightT + leftT;
        backLeftPower = left + rightT - leftT;
        frontRightPower = right + rightT - leftT;
        backRightPower = right - rightT + leftT;

        reducePowers(Math.max(frontLeftPower, Math.max(backLeftPower, Math.max(frontRightPower, backRightPower))));

        if(gamepad2.left_bumper) spikeyBoi.setPosition(spikeyOpen);
        if(gamepad2.right_bumper) spikeyBoi.setPosition(spikeyClosed);

        if(gamepad2.dpad_up) grabbyBoi.setPosition(grabbyOpen);
        if (gamepad2.dpad_down) grabbyBoi.setPosition(grabbyClosed);

        if(Math.abs(gamepad2.left_stick_y) > .1 ) {
            if(gamepad2.left_stick_y < 0) climbyBoi.setPower(gamepad2.left_stick_y * .8);
            if(gamepad2.left_stick_y > 0) climbyBoi.setPower(gamepad2.left_stick_y * .3);
        } else {
            climbyBoi.setPower(0);
        }
        frontRight.setPower((frontRightPower*turbo)/10);
        backRight.setPower((backRightPower*turbo)/10);
        frontLeft.setPower((frontLeftPower*turbo)/10);
        backLeft.setPower((backLeftPower*turbo)/10);

//        telemetry.addData("error", error);
//        telemetry.addData("up", gamepad1.dpad_up);
//        telemetry.addData("down", gamepad1.dpad_down);
        telemetry.addData("X: ", globalX);
        telemetry.addData("Y: ", globalY);
        telemetry.update();
    }

    public double ticksToInches(double x) {
        return 4 * Math.PI * x/488;
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