package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@SuppressWarnings("WeakerAccess")
@TeleOp(name="teleopCygnus")
public class teleopCygnus extends OpMode {

    //init vars
    private float left, right, leftT, rightT, frontLeftPower, backLeftPower, frontRightPower, backRightPower;
    private DcMotor frontRight, frontLeft, backRight, backLeft, liftL, liftR, encoderWheelY, encoderWheelX;
    private Servo grab;
    private DistanceSensor distSensor;

    boolean dpu2, dpd2 = false;

    int liftTarget = 0;
    int liftPos = 0;
    double liftPower = 0;
    double distanceL;
    double integralL;
    double derivativeL;
    double deltaT;
    double lastTime = 0;
    int deltaDL;
    int lastEncoderL = 0;
    double L_P_COEFF = 0.008;//0.008
    double L_I_COEFF = 0.07;
    double L_D_COEFF = 0.0004;
    double turboL;


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
        liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        grab.setPosition(0.5);
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

        turboL = 1;
        if(gamepad2.left_bumper||gamepad2.right_bumper) turboL++;

        if(Math.abs(gamepad2.left_stick_y)<0.03) {
            liftPos = getCurrentLiftPosition();

            if (gamepad2.dpad_up && !dpu2) {
                if((liftTarget%280)<(280/4)){
                    liftTarget+=280;
                }
                liftTarget+=((liftTarget%280));
                dpu2 = true;
            }else if (gamepad2.dpad_down && !dpd2) {
                if((liftTarget%280)>((280*3)/4)){
                    liftTarget-=280;
                }
                liftTarget-=(280-(liftTarget%280));
                dpd2 = true;
            } else if (!gamepad2.dpad_down && !gamepad2.dpad_up){
                dpu2 = false;
                dpd2 = false;
            }

            if(liftTarget<=0 || gamepad2.x){
                liftTarget = 0;
            } else if(liftTarget>2500){
                liftTarget=2500;
            }

            distanceL = liftTarget-liftPos;

            if(liftPos<280&&liftTarget<280){
                //liftPower=0;
                liftPower = (distanceL * L_P_COEFF);
            } else {
                liftPower = (distanceL * L_P_COEFF);
            }
        } else {
            liftPower = -(gamepad2.left_stick_y/turboL);
            liftTarget = getCurrentLiftPosition();
            dpu2 = false;
            dpd2 = false;
        }
        liftL.setPower(liftPower);
        liftR.setPower(liftPower);

        if(gamepad2.a){
            grab.setPosition(0.76);
        } else if (gamepad2.y){
            grab.setPosition(0);
        } else if (gamepad2.b){
            grab.setPosition(0.5);
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

    private int getCurrentLiftPosition() {return Math.abs((liftL.getCurrentPosition()+liftR.getCurrentPosition())/2);} //280 ticks per floor

    @Override
    public void stop() {}
}