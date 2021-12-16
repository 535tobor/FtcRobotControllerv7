package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@Disabled
public class HwMapIter extends OpMode {
    @Override
    public void init() {

    }
    public DcMotor fl, fr, br, bl, arm, extend;
    public Servo pincer;
    public CRServo carouSpin, pincer1;
    public int rotations = 0;
    public BNO055IMU imu;
    public Orientation gyroAngles;
    public double desiredRobotHeading;
    public void initHwMap(){
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
        carouSpin = hardwareMap.crservo.get("carouSpin");
        pincer = hardwareMap.servo.get("pincer");
        pincer1 = hardwareMap.crservo.get("pincer1");
        arm = hardwareMap.dcMotor.get("arm");
        extend = hardwareMap.dcMotor.get("extend");
        //test bot reverse statements
        //fr.setDirection(DcMotor.Direction.REVERSE);
        //br.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        //servo = hardwareMap.servo.get("servo");
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
    public double getIntegratedHeading() {
        if(desiredRobotHeading - (rotations * 360 + gyroAngles.firstAngle) > 200) {
            rotations++;
        }
        else if(desiredRobotHeading - (rotations * 360 + gyroAngles.firstAngle) < -200) {
            rotations--;
        }

        return (rotations * 360 + gyroAngles.firstAngle);
    }
}
