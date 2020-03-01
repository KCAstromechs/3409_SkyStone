package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp(name="teleop gabe solo")
public class teleopGabrielSolo extends OpMode {

    //init vars
    private float left, right, leftT, rightT, frontLeftPower, backLeftPower, frontRightPower, backRightPower;
    private DcMotor frontRight, frontLeft, backRight, backLeft, lift, flip, encoderWheelY, encoderWheelX;
    private Servo foundRight, foundLeft, mainFlop, subFlop, release;
    private DistanceSensor distSensor;
    private int turbo = 3;
    private int flopPos = 0;
    private boolean y, yLast, l12, l12Last, y2Last, x2Last, r12, r12Last = false;
    private int subFlopLast = 1;

    double ticksPerInchTetrix = ((1400)/(3 * Math.PI));


    @Override
    public void init() {
        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        encoderWheelY = hardwareMap.dcMotor.get("encoderWheelY");
        encoderWheelX = hardwareMap.dcMotor.get("encoderWheelX");
        lift = hardwareMap.dcMotor.get("lift");
        flip = hardwareMap.dcMotor.get("flip");

        foundLeft  = hardwareMap.servo.get("foundationLeft");
        foundRight = hardwareMap.servo.get("foundationRight");
        mainFlop   = hardwareMap.servo.get("mainFlop");
        subFlop    = hardwareMap.servo.get("subFlop");
        release    = hardwareMap.servo.get("release");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        distSensor = hardwareMap.get(DistanceSensor.class, "distSensor");

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderWheelY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderWheelX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderWheelY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderWheelX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
    }

    @Override
    public void loop() {
        turbo = 2;

        left = (Math.abs(gamepad1.right_stick_y) < 0.05) ? 0 : -gamepad1.right_stick_y;
        right = (Math.abs(gamepad1.left_stick_y) < 0.05) ? 0 : -gamepad1.left_stick_y;
        leftT = (Math.abs(gamepad1.left_trigger) < 0.05) ? 0 : gamepad1.left_trigger;
        rightT = (Math.abs(gamepad1.right_trigger) < 0.05) ? 0 : gamepad1.right_trigger;

        frontLeftPower = left + rightT - leftT;
        backLeftPower = left - rightT + leftT;
        frontRightPower = right - rightT + leftT;
        backRightPower = right + rightT - leftT;

        reducePowers(Math.max(frontLeftPower, Math.max(backLeftPower, Math.max(frontRightPower, backRightPower))));

        if (gamepad1.left_bumper) turbo ++;

        frontRight.setPower((frontRightPower*turbo)/3);
        backRight.setPower((backRightPower*turbo)/3);
        frontLeft.setPower((frontLeftPower*turbo)/3);
        backLeft.setPower((backLeftPower*turbo)/3);

        if(gamepad1.y && !yLast){
            if(y){
                foundRight.setPosition(0.75);
                foundLeft.setPosition(0);
                y = false;
            } else {
                foundRight.setPosition(0);
                foundLeft.setPosition(0.75);
                y = true;
            }
            yLast = true;
        } else if (!gamepad1.y && yLast) {
            yLast = false;
        }

        if(gamepad2.left_bumper && !l12Last){
            if(l12){
                release.setPosition(0);
                l12 = false;
            } else {
                release.setPosition(1);
                l12 = true;
            }
            l12Last = true;
        } else if (!gamepad2.left_bumper && l12Last) {
            l12Last = false;
        }

        flip.setPower(gamepad2.right_stick_y);

        lift.setPower(gamepad2.left_stick_y);

        if (gamepad1.dpad_up){
            lift.setPower(1);
        } else if (gamepad1.dpad_down) {
            lift.setPower(-1);
        } else {
            lift.setPower(0);
        }

        if(gamepad1.right_bumper && !r12Last && flopPos!=0){
            if(r12){
                subFlop.setPosition(0.35);
                subFlopLast = 1;
                r12 = false;
            } else {
                subFlop.setPosition(0.7);
                subFlopLast = 2;
                r12 = true;
            }
            r12Last = true;
        } else if (!gamepad1.right_bumper && r12Last) {
            r12Last = false;
        }

        if(gamepad1.a && !y2Last){
            if(flopPos != 2){
                flopPos ++;
                if(subFlopLast==2){
                    subFlop.setPosition(0.7);
                }
            }
            y2Last = true;
        } else if (!gamepad1.a && y2Last) {
            y2Last = false;
        }

        if(gamepad1.b && !x2Last){
            if(flopPos != 0){
                flopPos --;
            }
            x2Last = true;
        } else if (!gamepad1.b && x2Last) {
            x2Last = false;
        }

        if(flopPos<0) flopPos=0;
        if(flopPos>2) flopPos=2;

        if(flopPos==0){
            mainFlop.setPosition(0.7);
            subFlop.setPosition(0.35);
        } else if (flopPos==1) {
            mainFlop.setPosition(0.15);//.35
        } else {
            mainFlop.setPosition(0);
        }

        telemetry.addData("lift", lift.getCurrentPosition());
        telemetry.addData("flip", flip.getCurrentPosition());
        telemetry.addData("range", String.format("%.01f in", distSensor.getDistance(DistanceUnit.INCH)));
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