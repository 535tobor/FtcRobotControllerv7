package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "DeliverRedWarehouse")
//@Disabled
public class DeliverBoxRedWH extends  HwMap{
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode()
    {
        side = "right";
        initHwMap();
        while(!isStarted()&&!isStopRequested())
        {
            updateGyro();
            telemetry.addData("", pincer.getPosition());
            telemetry.update();
        }
        if(opModeIsActive())
        {
            deliverBox(45, 45);
            //if red warehouse:
//            motorRTP(extend, extend.getCurrentPosition()-10000, -.7);
//            encoderDrive(-.7, -.7, 8, 5, 1);
//            turn(45, 4);
//            encoderDrive(-.7, -.7, 25, 6, 1);
            /*
                    - back up for a few seconds or inches to get arm out of way
                    - turn so back of robot is facing the warehouse
                    - back into parking
             */

        }
    }
}

