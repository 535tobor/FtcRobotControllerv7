package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Carousel")
@Disabled
public class CarouselThing extends HwMap {
ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode(){
        initHwMap();

        while(!isStarted()&&!isStopRequested()) //init not run
        {

        }
        if(opModeIsActive()) //run
        {
            spin(5.0, 1);

        }


    }

}
