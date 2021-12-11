package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Carousel")

public class CarouselThing extends HwMap {
ElapsedTime timer = new ElapsedTime();
public void spin(double length){
    while (timer.time() < length){
        carouSpin.setPower(1);
    }
    carouSpin.setPower(0);
}
    @Override
    public void runOpMode(){
        initHwMap();
        //setModeAll(DcMotor.RunMode.RUN_USING_ENCODER);
        //setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(!isStarted()&&!isStopRequested()) //init not run
        {
            updateGyro();
            //telemetry.addData("robo angle: ", getIntegratedHeading());

            telemetry.update();
        }
        if(opModeIsActive()) //run
        {
            spin(5.0);

        }


    }

}
