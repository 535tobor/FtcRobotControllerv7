package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Auto Cases")
@Disabled
public class AutoCases extends HwMap{
    int encoderCountsToCarousel;
    int carouselDeg = -45, parkDeg = -90;
    boolean selectionButtonPressed = false, buttonPressed = false, isCarousel = false, isReady = false;
    int caseNum = 0, startingDelay = 0;
    String color = "";
    @Override
    public void runOpMode()
    {
        initHwMap();
        while(!isStarted()&&!isStopRequested()) //init
        {
            updateGyro();
            telemetry.update();
            if(!isReady) {
                if (gamepad1.a && !selectionButtonPressed) {
                    caseNum++;//toggle up from case 0 to case...
                    selectionButtonPressed = !selectionButtonPressed;
                } else if (gamepad1.y && !selectionButtonPressed) {
                    caseNum--;//toggle down from current case
                    selectionButtonPressed = !selectionButtonPressed;
                } else if (!gamepad1.a && !gamepad1.y && selectionButtonPressed) {
                    selectionButtonPressed = !selectionButtonPressed;
                }
                switch (caseNum) {
                    case 0:  //
                        if (gamepad1.b) {
                            color = "red";
                        }
                        if (gamepad1.x) {
                            color = "blue";
                        }

                        telemetry.addData("> Set starting color ", "Current Value: " + color);
                        telemetry.addData("B Button = red", "X Button = blue");
                        break;

                    case 1://first case, starting position because there are different movements we will do
                        if (gamepad1.b) {
                            side = "right";
                        }//starting position will be on the right
                        if (gamepad1.x) {
                            side = "left";
                        }//starting position will be on the left (only label not assigning values yet)

                        //left is closer to carousel and we will likely always go for the carousel spinner
                        //right is likely more collection and placement
                        telemetry.addData("> Set Start Position (according to tower)", " Current Value: " + side);
                        telemetry.addData("B Button = Right", "X Button = Lt");
                        break;
                    case 2:
                        if (gamepad1.b) {
                            isReady = true;
                        }
                        if (gamepad1.x) {
                            isReady = false;
                        }
                        telemetry.addData("> Are the cases ready? ", " Current Value: " + isReady);
                        telemetry.addData("B Button = True", "X Button = false");
                        break;
                    default:
                        caseNum = Math.max(caseNum, 0);//if caseNum is less than 0, set it back to 0 so you are not in the neagtive, if greater than 0, then caseNum = caseNum
                        caseNum = Math.min(caseNum, 2);// if caseNum exceeds the number of cases, set it back to the limit, else set it to caseNum
                        break;
                }
            }
//            if((color.equals("red")&&side.equals("left"))||(color.equals("blue")&&side.equals("right")))
//            {
//                isCarousel = true;
//            }
            runtime.reset();
            while(runtime.time()<3)
            {
                distanceArrayAverage();
                break;
            }
            barcodeDetect();
            updateGyro();
            telemetry.update();
        }
        if(opModeIsActive())
        {
            deliverBox();
        }
    }
}
