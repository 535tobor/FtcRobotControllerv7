package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Encoder")

public class EncoderDrive extends HwMap {
    int encoderCountsToCarousel;
    int inchingForward;


    @Override
    public void runOpMode(){
        initHwMap();
        while(!isStarted()&&!isStopRequested()) //init not run
        {
            updateGyro();
            telemetry.addData("robo angle: ", getIntegratedHeading());
            telemetry.update();
        }
        if(opModeIsActive()) //run
        {
            turn(90);
            //setTargetPositionAll(inchingForward);//drive forward a little so you have space to pivot
            //pivot
            //90degree turn
            //extend? arm or rotate arm on axle
            //idk if it will be accurate enough to have set positions on encoders and drive em but for now yes it is good
            //run ball servo to spin carousel\
            //distance sensor on ball
        }
    }
    public void setRTPAll()
    {
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setModeAll()
    {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setTargetPositionAll(int encoderCounts) //JUST FOR ONE MOTOR AT A TIME IF LINEAR
    {
        fl.setTargetPosition(encoderCounts);
        fr.setTargetPosition(encoderCounts);
        bl.setTargetPosition(encoderCounts);
        br.setTargetPosition(encoderCounts);
    }
    public void setPowerAll(double power)
    {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }
    public void moveForward(int encoderCounts, double power)
    {
        setModeAll();
        setTargetPositionAll(encoderCounts);
        setPowerAll(power);
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
}
