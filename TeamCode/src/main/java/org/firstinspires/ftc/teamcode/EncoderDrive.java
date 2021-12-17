package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Encoder")

public class EncoderDrive extends HwMap {
    int encoderCountsToCarousel;
    int carouselDeg = -45, parkDeg = -90;
    boolean selectionButtonPressed = false, buttonPressed = false, boxOnHub = false, isSpinCarousel=false, collectElems = false, currBool = false;
    int caseNum = 0, startingDelay = 0;
    String startPos = "", promptStr = "", color = "";



    @Override
    public void runOpMode()
    {
        initHwMap();
        setModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //changed, should work???
        while(!isStarted()&&!isStopRequested()) //init not run
        {
            updateGyro();
            telemetry.update();
            if(gamepad1.a&&!selectionButtonPressed){
                caseNum++;//toggle up from case 0 to case...
                selectionButtonPressed=!selectionButtonPressed;
            }
            else if(gamepad1.y&&!selectionButtonPressed){
                caseNum--;//toggle down from current case
                selectionButtonPressed=!selectionButtonPressed;
            }
            else if(!gamepad1.a&&!gamepad1.y&&selectionButtonPressed){
                selectionButtonPressed=!selectionButtonPressed;
            }
            switch(caseNum){
                case 0:  //
                    if(gamepad1.b) {color = "red";}
                    if(gamepad1.x) {color = "blue";}

                    telemetry.addData("> Set starting color ", "Current Value: " + color );
                    telemetry.addData("B Button = red", "X Button = blue");
                    break;

                case 1://first case, starting position because there are different movements we will do
                    if(gamepad1.b) {startPos = "Rt";}//starting position will be on the right
                    if(gamepad1.x) {startPos = "Lt"; promptStr = "Do you want to spin the carousel? ";}//starting position will be on the left (only label not assigning values yet)

                    //left is closer to carousel and we will likely always go for the carousel spinner
                    //right is likely more collection and placement
                    telemetry.addData("> Set Start Position", "Current Value: " + startPos);
                    telemetry.addData("B Button = Right", "X Button = Lt");
                    break;

                case 2: // carousel spin(if start left), collect more elements(if right)
                    if(startPos.equals("Rt"))
                    {
                        if(gamepad1.b) {collectElems = true;}
                        if(gamepad1.x) {collectElems = false;}
                        currBool = collectElems;

                    }
                    else if(startPos.equals("Lt"))
                    {
                        if(gamepad1.b) {isSpinCarousel = true;}
                        if(gamepad1.x) {isSpinCarousel = false;}
                        currBool = isSpinCarousel;
                    }


                    telemetry.addData("> " + promptStr, " Current Value: " + currBool);
                    telemetry.addData("B Button = Yes", "X Button = No");
                    break;

                case 3: //add start delay
                    if(gamepad1.b && !buttonPressed) {
                        startingDelay++;//toggle up the delay time
                        buttonPressed = !buttonPressed;
                    } else if(gamepad1.x && !buttonPressed) {
                        startingDelay--;//toggle down the delay time
                        buttonPressed = !buttonPressed;
                    } else if (!gamepad1.b && !gamepad1.x && buttonPressed) {
                        buttonPressed = !buttonPressed;
                    }
                    telemetry.addData("> Set Start Delay", "Current Value: " + startingDelay + " seconds");
                    telemetry.addData("B Button to increase", "X Button to decrease");
                    break;
                default:
                    caseNum = Math.max(caseNum, 0);//if caseNum is less than 0, set it back to 0 so you are not in the neagtive, if greater than 0, then caseNum = caseNum
                    caseNum = Math.min(caseNum, 3);// if caseNum exceeds the number of cases, set it back to the limit, else set it to caseNum
                    break;
            }
        }
        if(opModeIsActive()) //run
        {
            if(color.equals("red"))
            {
                if(startPos.equals("Lt"))
                {
                    carousel(carouselDeg);
                }
                else if(startPos.equals("Rt"))
                {
                    warehousePark(parkDeg);
                }
            }
            else if(color.equals("blue"))
            {
                if(startPos.equals("Rt"))
                {
                    carousel(-carouselDeg);
                }
                else if(startPos.equals("Lt"))
                {
                    warehousePark(-parkDeg);
                }
            }
        }
    }
//    public void deliverCargo()
//    {
//
//    }
    public void warehousePark(int degrees)
    {
        encoderDrive(.8, .8, 10, 5, 1);// move forward to not run into wall
        turn(degrees);// turn towards the warehouse
        sleep(250);
        encoderDrive(.8, .8, 16, 7, 1);// drive into warehouse
    }

    public void carousel(int degrees)
    {
        encoderDrive(.8, .8, 10, 5, 1);// move forward to not run into wall
        turn(degrees);// turn towards the right so the back of the bot carou spinner is facing the carousel
        sleep(250);
        encoderDrive(-.8, -.8, 7, 7, 1);// backward towards carousel
        spin(5); //spin the carousel
        //park
        encoderDrive(.8, .8, 10, 5, 1);


        setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
