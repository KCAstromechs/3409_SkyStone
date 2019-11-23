package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class M1TeleOp extends OpMode {
    float left, right, leftTrigger, rightTrigger, frontLeftPower, frontRightPower, backLeftPower, backRightPower;

    float foundationLeftRetracted = 0;
    float foundationLeftDeployed = 1;
    float foundationRightRetracted = 0;
    float foundationRightDeployed = 1;
    float mainFlopRetracted = 0;
    float mainFlopDeployed = 1;
    float subFlopRetracted = 0;
    float subFlopDeployed = 1;
    float releaseRetracted = 0;
    float releaseDeployed = 1;

    @Override
    public void init() {
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor flip = hardwareMap.dcMotor.get("flip");
        Servo foundationLeft = hardwareMap.servo.get("foundationLeft");
        Servo foundationRight = hardwareMap.servo.get("foundationRight");
        Servo mainFlop = hardwareMap.servo.get("mainFlop");
        Servo subFlop = hardwareMap.servo.get("subFlop");
        Servo release = hardwareMap.servo.get("release");


        foundationLeft.setPosition(foundationLeftRetracted);
        foundationRight.setPosition(foundationRightRetracted);
        mainFlop.setPosition(1);
    }

    @Override
    public void loop() {
        left = (Math.abs(gamepad1.left_stick_y) < 0.05) ? 0 : gamepad1.left_stick_y;
        right = (Math.abs(gamepad1.right_stick_y) < 0.05) ? 0 : gamepad1.right_stick_y;
        leftTrigger = (Math.abs(gamepad1.left_trigger) < 0.05) ? 0 : gamepad1.left_trigger;
        rightTrigger = (Math.abs(gamepad1.right_trigger) < 0.05) ? 0 : gamepad1.right_trigger;


        frontLeftPower = left - rightTrigger + leftTrigger;
        backLeftPower = left + rightTrigger - leftTrigger;
        frontRightPower = right + rightTrigger - leftTrigger;
        backRightPower = right - rightTrigger + leftTrigger;

        reducePowers(Math.max(frontLeftPower, Math.max(backLeftPower, Math.max(frontRightPower, backRightPower))));

    }
    private void reducePowers(float power) {
        power = Math.abs(power);
        if (power > 1.0) {

            float multiplier = 1/power;

            frontLeftPower *= multiplier;
            frontRightPower *= multiplier;
            backLeftPower *= multiplier;
            backRightPower *= multiplier;
        }
    }
}
