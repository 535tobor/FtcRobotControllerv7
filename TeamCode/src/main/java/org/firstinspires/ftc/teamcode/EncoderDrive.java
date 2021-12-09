package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Encoder")

public class EncoderDrive extends HwMap {
    int encoderCountsToCarousel;
    int carouselDeg = 110;

    @Override
    public void runOpMode(){
        initHwMap();
        //setModeAll(DcMotor.RunMode.RUN_USING_ENCODER);
        setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(!isStarted()&&!isStopRequested()) //init not run
        {
            updateGyro();
            //telemetry.addData("robo angle: ", getIntegratedHeading());
            telemetry.update();
        }
        if(opModeIsActive()) //run
        {
            encoderDri(.6, .6, 10, 10, 2);
            //turn(carouselDeg);
            //encoderDri(.6, .6, 10, 5, 2);

            //pivot
            //90degree turn
            //extend? arm or rotate arm on axle
            //idk if it will be accurate enough to have set positions on encoders and drive em but for now yes it is good
            //run ball servo to spin carousel\
            //distance sensor on ball
        }


    }
    public void encoderDri(double Lspeed, double Rspeed, double Inches, double timeoutS, double rampup){
        int newLeftTarget, newRightTarget, currLeftAvg, currRightAvg;
        // Ensure that the opmode is still active
        // Determine new target position, and pass to motor controller we only do this in case the encoders are not totally zero'd
        currLeftAvg = (fl.getCurrentPosition() + bl.getCurrentPosition()) / 2;
        currRightAvg = (fr.getCurrentPosition() + br.getCurrentPosition()) / 2;
        newLeftTarget = currLeftAvg + (int) (Inches * COUNTS_PER_INCH);
        newRightTarget = currRightAvg + (int) (Inches * COUNTS_PER_INCH);
        // reset the timeout time and start motion.
        runtime.reset();
        // keep looping while we are still active, and there is time left, and neither set of motors have reached the target
        while ((runtime.seconds() < timeoutS) &&
                (Math.abs(currLeftAvg) < newLeftTarget &&
                        Math.abs(currRightAvg) < newRightTarget)) {
            currLeftAvg = (fl.getCurrentPosition() + bl.getCurrentPosition()) / 2;
            currRightAvg = (fr.getCurrentPosition() + br.getCurrentPosition()) / 2;
            double rem = (Math.abs(fl.getCurrentPosition()) + Math.abs(bl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition()) + Math.abs(br.getCurrentPosition())) / 4;
            double NLspeed;
            double NRspeed;
            //To Avoid spinning the wheels, this will "Slowly" ramp the motors up over
            //the amount of time you set for this SubRun
            double R = runtime.seconds();
            if (R < rampup) {
                double ramp = R / rampup;
                NLspeed = Lspeed * ramp;
                NRspeed = Rspeed * ramp;
            }
//Keep running until you are about two rotations out
            else if (rem > (2240)) {
                NLspeed = Lspeed;
                NRspeed = Rspeed;
            }
            //start slowing down as you get close to the target
            else if (rem > (400) && (Lspeed * .2) > .1 && (Rspeed * .2) > .1) {
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
        //Note: This is outside our while statement, this will only activate once the time, or distance has been met
        setPowerZero();
        // show the driver how close they got to the last target
        telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
        telemetry.addData("Path2", "Running at %7d :%7d", fl.getCurrentPosition(), fr.getCurrentPosition());
        telemetry.update();
        //setting resetC as a way to check the current encoder values easily
        double resetC = ((Math.abs(fl.getCurrentPosition()) + Math.abs(bl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition()) + Math.abs(fr.getCurrentPosition())));
        //Get the motor encoder resets in motion
        setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //keep waiting while the reset is running
        while (Math.abs(resetC) > 0) {
            resetC = ((Math.abs(fl.getCurrentPosition()) + Math.abs(bl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition()) + Math.abs(fr.getCurrentPosition())));
            idle();
        }
        // switch the motors back to RUN_USING_ENCODER mode
        setModeAll(DcMotor.RunMode.RUN_USING_ENCODER);
        //give the encoders a chance to switch modes.
        //  sleep(250);   // optional pause after each move
    }
}
