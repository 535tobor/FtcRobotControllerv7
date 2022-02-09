package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "DeliverBlueWarehouse")
//@Disabled
public class DeliverBoxBlueWH extends  HwMap{
    ElapsedTime timer = new ElapsedTime();
    int armCts = 0;
    String level = "";
    @Override
    public void runOpMode()
    {
        side = "left";
        initHwMap();
        while(!isStarted()&&!isStopRequested())
        {
            level = barcodeDetect(); // loop in initialization, keep checking
            updateGyro();
            telemetry.addData("", pincer.getPosition());
            telemetry.update();
        }
        if(opModeIsActive())
        {
            deliverBox(timer, -45, 45);

            //if blue warehouse:
//            motorRTP(extend, extend.getCurrentPosition()-10000, -.7);
//            encoderDrive(-.7, -.7, 8, 5, 1);
//            turn(-45, 4);
//            encoderDrive(-.7, -.7, 25, 6, 1);
//            /*
//                    - back up for a few seconds or inches to get arm out of way
//                    - (negative) turn so back of robot is facing the warehouse
//                    - back into parking
//             */

        }
    }
}

