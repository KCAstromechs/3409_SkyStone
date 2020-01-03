package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="teleopCygnus")
public class teleopCygnus extends OpMode {

    //init vars
    private float left, right, leftT, rightT, frontLeftPower, backLeftPower, frontRightPower, backRightPower;
    private DcMotor frontRight, frontLeft, backRight, backLeft, liftL, liftR, encoderWheelY, encoderWheelX;
    private Servo grab;
    private DistanceSensor distSensor;

    double ticksPerInchTetrix = ((1400)/(3 * Math.PI));


    @Override
    public void init() {
        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        encoderWheelY = hardwareMap.dcMotor.get("encoderWheelY");
        encoderWheelX = hardwareMap.dcMotor.get("encoderWheelX");
        liftL = hardwareMap.dcMotor.get("liftL");
        liftR = hardwareMap.dcMotor.get("liftR");

        grab = hardwareMap.servo.get("grab");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        liftR.setDirection(DcMotor.Direction.REVERSE);

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
    }

    @Override
    public void loop() {

        left = (Math.abs(gamepad1.left_stick_y) < 0.05) ? 0 : -gamepad1.left_stick_y;
        right = (Math.abs(gamepad1.right_stick_y) < 0.05) ? 0 : -gamepad1.right_stick_y;
        leftT = (Math.abs(gamepad1.left_trigger) < 0.05) ? 0 : gamepad1.left_trigger;
        rightT = (Math.abs(gamepad1.right_trigger) < 0.05) ? 0 : gamepad1.right_trigger;

        frontLeftPower = left + rightT - leftT;
        backLeftPower = left - rightT + leftT;
        frontRightPower = right - rightT + leftT;
        backRightPower = right + rightT - leftT;

        reducePowers(Math.max(frontLeftPower, Math.max(backLeftPower, Math.max(frontRightPower, backRightPower))));

        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);

        liftL.setPower(-gamepad2.left_stick_y);
        liftR.setPower(-gamepad2.left_stick_y);

        if(gamepad2.a){
            grab.setPosition(0);
        } else if (gamepad2.b){
            grab.setPosition(1);
        }

        telemetry.addData("liftL", liftL.getCurrentPosition());
        telemetry.addData("liftR", liftR.getCurrentPosition());
        telemetry.addData("odometer Y", encoderWheelY.getCurrentPosition());
        telemetry.addData("odometer X", encoderWheelX.getCurrentPosition());
        telemetry.addData("inches Y", encoderWheelY.getCurrentPosition()/ticksPerInchTetrix);
        telemetry.addData("inches X", encoderWheelX.getCurrentPosition()/ticksPerInchTetrix);
        telemetry.update();
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
    public void stop() {}
}