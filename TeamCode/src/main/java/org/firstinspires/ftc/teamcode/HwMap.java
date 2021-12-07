package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class HwMap extends LinearOpMode {
    public DcMotor fl, fr, br, bl;
    public int rotations = 0;
    public BNO055IMU imu;
    public Orientation gyroAngles;
    public double desiredRobotHeading;
    public final double WHEEL_DIA = 5.65;
    public final double COUNTS_PER_INCH = 1120/(WHEEL_DIA * 3.14);
    public ElapsedTime runtime = new ElapsedTime();

    public void initHwMap(){
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        //bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    @Override
    public void runOpMode()  {

    }
    public void updateGyro()
    {
        gyroAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
    public double getIntegratedHeading() {
        if(desiredRobotHeading - (rotations * 360 + gyroAngles.firstAngle) > 200) {
            rotations++;
        }
        else if(desiredRobotHeading - (rotations * 360 + gyroAngles.firstAngle) < -200) {
            rotations--;
        }

        return (rotations * 360 + gyroAngles.firstAngle);
    }

    public void setRTPAll()
    {
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setModeAll(DcMotor.RunMode mode)
    {
        fl.setMode(mode);
        fr.setMode(mode);
        bl.setMode(mode);
        br.setMode(mode);
    }
    public void setTargetPositionAll(int flEncoderCts, int frEncoderCts, int blEncoderCts, int brEncoderCts) //JUST FOR ONE MOTOR AT A TIME IF LINEAR
    {
        fl.setTargetPosition(flEncoderCts);
        fr.setTargetPosition(frEncoderCts);
        bl.setTargetPosition(blEncoderCts);
        br.setTargetPosition(brEncoderCts);
    }
    public void setPowerAll(double power)
    {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }
    public void setPowerZero()
    {
        setPowerAll(0);
    }
    public void turn(int degrees) //from current
    {
        double initAngle = getIntegratedHeading();
        double desiredAngle = initAngle + degrees;
        while(opModeIsActive()&&(getIntegratedHeading()>( desiredAngle + 3)||getIntegratedHeading()<(desiredAngle- 3)))
        {//left is pos
            updateGyro();
            if(degrees>0)
            {
                fl.setPower(-.7);
                fr.setPower(.7);
                bl.setPower(-.7);
                br.setPower(.7);
            }
            else
            {
                fl.setPower(.7);
                fr.setPower(-.7);
                bl.setPower(.7);
                br.setPower(-.7);
            }
        }
        setPowerAll(0);
    }
    public void encoderDrive(double motorPower, int inches, int timeoutS)
    {
        int newFLTarget;
        int newFRTarget;
        int newBLTarget;
        int newBRTarget;

        if (opModeIsActive()) {

            newFLTarget = fl.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newFRTarget = fr.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newBLTarget = bl.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newBRTarget = br.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

            setTargetPositionAll(newFLTarget, newFRTarget, newBLTarget, newBRTarget); //set target position

            setRTPAll(); //set run to position for all motors

            runtime.reset();
            setPowerAll(motorPower);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy())) {

                telemetry.addData("Path1",  "Running to %7d :%7d", newFLTarget,  newFRTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        fl.getCurrentPosition(),
                        fr.getCurrentPosition(),
                        bl.getCurrentPosition());
                telemetry.update();
            }

            setPowerZero();

            setModeAll(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

}
