package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@TeleOp(name = "Drive")
@Disabled
public class Demo extends OpMode {

    DcMotor br, bl, fr, fl, lift;
    Servo leftGripServo, rightGripServo;
    double desiredRobotHeading;
    double fieldReference;
    double x, y, joystickAngle, joystickAngle360;
    double driveSpeed = 0;
    double flPower, frPower, brPower, blPower;
    double maxMotorPower;
    double driveRotation;
    int rotations = 0;
    BNO055IMU imu;
    Orientation gyroAngles;
    boolean liftUpRangeIsValid;
    boolean liftDownRangeIsValid;
    boolean isTurn = false;
    @Override
    public void init() {
        br = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        fl= hardwareMap.get(DcMotor.class, "fl");
        lift = hardwareMap.get(DcMotor.class, "lift");
        leftGripServo = hardwareMap.get(Servo.class, "leftGrip");
        rightGripServo = hardwareMap.get(Servo.class, "rightGrip");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //fl.setDirection(DcMotorSimple.Direction.REVERSE);
        //bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        liftUpRangeIsValid = lift.getCurrentPosition()<-20;//change value
        liftDownRangeIsValid = lift.getCurrentPosition()>-4000;

        gyroAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        if (gamepad1.right_trigger > .05 || gamepad1.left_trigger > .05) {
//            //driveRotation = gamepad1.left_trigger - gamepad1.right_trigger;
//            desiredRobotHeading = getIntegratedHeading();
//        } else if (Math.abs(desiredRobotHeading - getIntegratedHeading()) > 5) {
//            //driveRotation = (desiredRobotHeading - getIntegratedHeading()) / (Math.abs(desiredRobotHeading - getIntegratedHeading())) * .05;
//        }
//        else {
//            //driveRotation = 0;
//        }
//        fieldReference = desiredRobotHeading;
//        //Positive values for x axis are joystick right
//        //Positive values for y axis are joystick down
//        y = Range.clip(gamepad1.right_stick_x,-1,1);
//        x = Range.clip(-gamepad1.right_stick_y,-1,1);
//        joystickAngle = Math.atan2(x,y);
//        // joystickAngle360 = joystickAngle >= 0 ? joystickAngle : (2*Math.PI) + joystickAngle;
//        joystickAngle360 = joystickAngle >= 0 ? joystickAngle : (2*Math.PI) + joystickAngle;
//        driveSpeed = Range.clip(Math.sqrt(y * y + x * x), -1, 1);
//
//        flPower = Math.cos(joystickAngle360 - Math.toRadians(fieldReference) + Math.PI / 4);
//        frPower = Math.sin(joystickAngle360 - Math.toRadians(fieldReference) + Math.PI / 4);
//        blPower = Math.sin(joystickAngle360 - Math.toRadians(fieldReference) + Math.PI / 4);
//        brPower = Math.cos(joystickAngle360 - Math.toRadians(fieldReference) + Math.PI / 4);
//
//        maxMotorPower = Math.max(Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.abs(blPower)), Math.abs(brPower));
//
//        //Ratio the powers for direction
//        flPower = flPower / maxMotorPower;
//        frPower = frPower / maxMotorPower;
//        blPower = blPower / maxMotorPower;
//        brPower = brPower / maxMotorPower;
//
//        flPower = driveSpeed * flPower ;//- driveRotation;
//        frPower = driveSpeed * frPower ;//+ driveRotation;
//        blPower = driveSpeed * blPower ;//- driveRotation;
//        brPower = driveSpeed * brPower ;//+ driveRotation;
//
//        maxMotorPower = Math.max(Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.abs(blPower)), Math.abs(brPower));
//        if (Math.abs(maxMotorPower) > 1) {
//            flPower = flPower / maxMotorPower;
//            frPower = frPower / maxMotorPower;
//            blPower = blPower / maxMotorPower;
//            brPower = brPower / maxMotorPower;
//        } else if (Math.abs(maxMotorPower) < .03) {
//            flPower = 0;
//            frPower = 0;
//            blPower = 0;
//            brPower = 0;
//        }
        if(gamepad1.right_trigger>.05)
        {
            flPower = -1;
            blPower = -1;
            frPower = 1;
            brPower = 1;
        }
        else if(gamepad1.left_trigger>.5)
        {
            flPower = 1;
            blPower = 1;
            brPower = -1;
            frPower = -1;
        }
        else if(gamepad1.right_stick_y>0)
        {
            flPower = 1;
            blPower = 1;
            brPower = 1;
            frPower = 1;
        }
        else if(gamepad1.right_stick_y<0)
        {
            flPower = -1;
            blPower = -1;
            brPower = -1;
            frPower = -1;
        }
        else
        {
            frPower = 0;
            flPower = 0;
            brPower = 0;
            blPower = 0;
        }
        fl.setPower(flPower*.4);
        fr.setPower(frPower*.4);
        bl.setPower(blPower*.4);
        br.setPower(brPower*.4);





        if(gamepad1.left_stick_y<0&&liftDownRangeIsValid)
        {
            lift.setPower(-.5);
        }
        else if(gamepad1.left_stick_y>0&&liftUpRangeIsValid)
        {
            lift.setPower(.5);
        }
        else{
            lift.setPower(0);
        }
        if(gamepad1.a)
        {
            leftGripServo.setPosition(0);
            rightGripServo.setPosition(1);
        }
        else if(gamepad1.b)
        {
            leftGripServo.setPosition(1);
            rightGripServo.setPosition(0);
        }
        // telemetry.addData("lift encoder position: ", lift.getCurrentPosition());
        telemetry.addData("Drive rotation: ", driveRotation);
    }
    private double getIntegratedHeading() {
        if(desiredRobotHeading - (rotations * 360 + gyroAngles.firstAngle) > 200) {
            rotations++;
        }
        else if(desiredRobotHeading - (rotations * 360 + gyroAngles.firstAngle) < -200) {
            rotations--;
        }

        return (rotations * 360 + gyroAngles.firstAngle);
    }
}
