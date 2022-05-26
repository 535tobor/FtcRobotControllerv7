package org.firstinspires.ftc.teamcode.leaguechamp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="EncoderDrive")
@Disabled
public class EncoderDrive extends HwMap {
    double inches = 0;
    @Override
    public void runOpMode()
    {
        initHwMap();
        setModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(!isStarted()&&!isStopRequested()) //init
        {
            updateGyro();
//            if(gamepad1.a)
//            {
//                inches+=.1;
//            }
//            telemetry.addData("current inches: ", inches);
            telemetry.update();
        }
        if(opModeIsActive()) //run
        {
            //carousel(39);
            driveByTime(-.6,.5);
            robotTurn(90, 5);
            driveByTime(.6,2);

            //warehousePark(-55);
        }
    }
    public void warehousePark(int degrees)
    {
        encoderDrive(.8, .8, 10, 5, 1);// move forward to not run into wall
        robotTurn(degrees, 5);// turn towards the warehouse
        sleep(250);
        encoderDrive(.8, .8, 16, 7, 1);// drive into warehouse
    }

    public void carousel(int degrees)
    {
        encoderDrive(.8, .8, 17, 5, 1);// move forward to not run into wall
        robotTurn(-degrees, 5);// turn towards the right so the back of the bot carou spinner is facing the carousel
        sleep(1000);
        encoderDrive(-.8, -.8, 15, 7, 1);// backward towards carousel
        spin(5, .45); //spin the carousel
        //park
        encoderDrive(.8, .8, 5, 5, 1); //drive forward to move away from spinner
        robotTurn(degrees,5);//turn to face parking
        encoderDrive(.8, .8,10, 7, 1); //park
        setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
//
//    public void preloadBoxScore(int forwardIN, int degrees, int hubIN, int armExtendEncoderCounts, int pincerOpenPos, int inchesToCarousel, int turnToParkDeg, int parkIN)
//    {
//        encoderDrive(.8, .8, forwardIN, 5, 1);// move forward to not run into wall
//        turn(degrees); // turn right to face hub and be ready to drive back into carousel spinner
//        encoderDrive(.8, .8, hubIN, 5, 1); // drive to hub
//        while(threshold(extend.getCurrentPosition(), armExtendEncoderCounts, 25)) //extend arm
//        {
//            extend.setPower(.7);
//        }
//        extend.setPower(0);
//        pincer.setPosition(pincerOpenPos); //open pincer to score box
//        //sleep statement?
//        encoderDrive(-.8, -.8, inchesToCarousel, 5, 1); //drive back to carousel
//        spin(4, .8);//run the carousel spinner to score the duck
//        carouSpin.setPower(0);
//        turn(turnToParkDeg); //turn to park
//        encoderDrive(.8, .8, parkIN, 5, 1); //drive forward and park
//    }


}
