package org.firstinspires.ftc.teamcode.ftcjavaguide; //location of class

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; //import information from an outside location

public class Example extends LinearOpMode {
    /*
        class statement (named "Example" so file must be named Example.java)
        extends means that this is a subclass of LinearOpMode
        a subclass inherits methods and variables from its superclass
        inheriting also involves being able to reference methods and variables from the superclass without objects
    */

    /*
    in a java program, rules must be followed in order for the program to run.
    rules include:
    - syntax such as ending each line with a semicolon(;)
    - initializing/declaring variables before using them
    */

    //data types
    // declaring a variable is setting a variable with a type and name
    // initializing a variable is setting a variable with a value it will hold
    int anInt = 10;
    /*
        an int is a value that is a whole number
     */
    float aFloat = 1.2f;
    /*
        a decimal with 6-7 digits of precision
     */
    double aDouble = 0.24853;
    /*
        a decimal with double the amount of precision
     */
    boolean aBoolean = false;
    /*
        a variable set to either true or false
     */
    String aString = "String";
    /*
        a variable that is a compilation of characters in quotations and can have words and sentences
     */
    String objString = new String("Object class");
    public int pubInt = 11;
    /*
        declaring a "public" variable means that if you reference the variable outside of the class, the variable is accessible and changeable
     */
    private int privInt = 12;
    /*
        a "private" variable is not accessible and changeable unless you use "getters" and "setters" [LINE283]
     */
    final int CONSTANT_VALUE = 1432;
    /*
        a "final" is a constant value that is not changeable in the program; good for calculations with a set value in the equation
     */
    @Override
    public void runOpMode() throws InterruptedException {

    }
}