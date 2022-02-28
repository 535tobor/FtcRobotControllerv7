package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
public class HwMapIterState extends OpMode {
    @Override
    public void init() {

    }
    public DcMotor fl, fr, br, bl, carouSpin;
    DcMotor intakeL, intakeR;
    public Servo pincer;
    public DistanceSensor dsL, dsR, dsBR;
    public int rotations = 0;
    public BNO055IMU imu;
    public Orientation gyroAngles;
    public double desiredRobotHeading;
    public boolean isIntakeRunning = false;

    public void initHwMap(){
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
        carouSpin = hardwareMap.dcMotor.get("carousel");
        pincer = hardwareMap.servo.get("pincer");
        dsL = hardwareMap.get(DistanceSensor.class, "ds");
        dsBR = hardwareMap.get(DistanceSensor.class, "dsBR");
        dsR = hardwareMap.get(DistanceSensor.class, "ds2");
        intakeL = hardwareMap.dcMotor.get("intakeL");
        intakeR = hardwareMap.dcMotor.get("intakeR");

        //test bot reverse statements
        //fr.setDirection(DcMotor.Direction.REVERSE);
        //br.setDirection(DcMotor.Direction.REVERSE);

        //competition bot reverse statements
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        //Gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    @Override
    public void loop() {

    }
    public void updateGyro()
    {
        gyroAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
    public double getRoboAngle() {
        if(desiredRobotHeading - (rotations * 360 + gyroAngles.firstAngle) > 200) {
            rotations++;
        }
        else if(desiredRobotHeading - (rotations * 360 + gyroAngles.firstAngle) < -200) {
            rotations--;
        }

        return (rotations * 360 + gyroAngles.firstAngle);
    }

    public void setModeAll(DcMotor.RunMode mode)
    {
        fl.setMode(mode);
        fr.setMode(mode);
        bl.setMode(mode);
        br.setMode(mode);
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

    public boolean threshold(int val, int otherval, int range)
    {
        return (val>( otherval + range)||val<( otherval - range));
    }
    public void launch(){
        intakeL.setPower(-1);
        intakeR.setPower(1);
        isIntakeRunning = true;
    }
    public void reverseLaunch()
    {
        intakeL.setPower(1);
        intakeR.setPower(-1);
        isIntakeRunning = true;
    }
    public void launchSetZero(){
        intakeL.setPower(0);
        intakeR.setPower(0);
        isIntakeRunning = false;
    }
}
