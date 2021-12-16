package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class HwMap extends LinearOpMode {

    public DcMotor fl, fr, br, bl;
    public CRServo carouSpin;
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
        carouSpin = hardwareMap.crservo.get("carouSpin");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


    }
    @Override
    public void runOpMode()  {

    }
    public void spin(double time)
    {
        while (runtime.time() < time){
            carouSpin.setPower(1);
        }
        carouSpin.setPower(0);
    }
    public void updateGyro()
    {
        gyroAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
    public double robotAngle() {
        if (desiredRobotHeading - (rotations * 360 + gyroAngles.firstAngle) > 200) {
            rotations++;
        } else if (desiredRobotHeading - (rotations * 360 + gyroAngles.firstAngle) < -200) {
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
    public void moveByTime(double speed, double time){
        while (runtime.time() > time ){
            setPowerAll(speed);
        }
        setPowerZero();
    }

    public void turn(int degrees) //from current
    {
        double initAngle = robotAngle();
        double desiredAngle = initAngle + degrees;
        while(opModeIsActive()&&(robotAngle()>( desiredAngle + 3)||robotAngle()<(desiredAngle- 3)))
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
    public void encoderDrive(double Lspeed, double Rspeed, double Inches, double timeoutS, double rampup){
        int newLeftTarget = (fl.getCurrentPosition() + bl.getCurrentPosition() )/2 + (int)(Inches * COUNTS_PER_INCH);
        int newRightTarget= (fr.getCurrentPosition() + br.getCurrentPosition() )/2 + (int)(Inches * COUNTS_PER_INCH);
        // Ensure that the opmode is still active
        // Determine new target position, and pass to motor controller we only do this in case the encoders are not totally zero'd
        // reset the timeout time and start motion.
        runtime.reset();
        // keep looping while we are still active, and there is time left, and neither set of motors have reached the target
        while ( opModeIsActive() && (runtime.seconds() < timeoutS) &&
                (Math.abs(fl.getCurrentPosition() + bl.getCurrentPosition()) /2 < newLeftTarget  &&
                        Math.abs(fr.getCurrentPosition() + br.getCurrentPosition())/2 < newRightTarget)) {
            double rem = (Math.abs(fl.getCurrentPosition()) + Math.abs(bl.getCurrentPosition())+Math.abs(fr.getCurrentPosition()) + Math.abs(br.getCurrentPosition()))/4;
            double NLspeed;
            double NRspeed;
            boolean tele = (Math.abs(fl.getCurrentPosition() + bl.getCurrentPosition()) /2 < newLeftTarget  &&
                    Math.abs(fr.getCurrentPosition() + br.getCurrentPosition())/2 < newRightTarget);
            boolean why = (runtime.seconds() < timeoutS);
            telemetry.addData("time bool: "+why+ " cur Pos bool: ",tele);
            telemetry.update();
            //To Avoid spinning the wheels, this will "Slowly" ramp the motors up over
            //the amount of time you set for this SubRun
            double R = runtime.seconds();
            if (R < rampup) {
                double ramp = R / rampup;
                NLspeed = Lspeed * ramp;
                NRspeed = Rspeed * ramp;
            }
//Keep running until you are about two rotations out
            else if(rem > (2000) )
            {
                NLspeed = Lspeed;
                NRspeed = Rspeed;
            }
            //start slowing down as you get close to the target
            else if(rem > (400) && (Lspeed*.2) > .1 && (Rspeed*.2) > .1) {
                NLspeed = Lspeed * (rem / 2000);
                NRspeed = Rspeed * (rem / 2000);
            }
            //minimum speed
            else {
                NLspeed = Lspeed * .2;
                NRspeed = Rspeed * .2;

            }
            //Pass the seed values to the motors
            fl.setPower(NLspeed);
            bl.setPower(NLspeed);
            fr.setPower(NRspeed);
            br.setPower(NRspeed);
        }
        // Stop all motion;
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}
