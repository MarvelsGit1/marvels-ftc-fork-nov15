package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static java.lang.Thread.sleep;


import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class BaseMotorMethods{
    private ElapsedTime     runtime = new ElapsedTime();
    private DcMotorEx arm;
    private DcMotorEx frontleft;
    private DcMotorEx frontright;
    private DcMotorEx backleft;
    private DcMotorEx backright;
    private int timeout = 10; //seconds until command abort

    public LinearOpMode parent;
    public Telemetry telemetry;

    //BACKEND SETUP
    public BaseMotorMethods(HardwareMap hardwareMap) {
        frontleft = hardwareMap.get(DcMotorEx.class, "frontleft");
        frontright = hardwareMap.get(DcMotorEx.class, "frontright");
        backleft = hardwareMap.get(DcMotorEx.class, "backleft");
        backright = hardwareMap.get(DcMotorEx.class, "backright");
        arm = hardwareMap.get(DcMotorEx.class, "arm");

        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setMotorMode(DcMotor.RunMode mode){
        frontleft.setMode(mode);
        frontright.setMode(mode);
        backleft.setMode(mode);
        backright.setMode(mode);
    }

    public void setMotorPosition(int pos1, int pos2, int pos3, int pos4){
        frontleft.setTargetPosition(pos1); //set encoder ticks target
        frontright.setTargetPosition(pos2);
        backleft.setTargetPosition(pos3);
        backright.setTargetPosition(pos4);
    }

    public void setMotorPower(double speed1, double speed2, double speed3, double speed4){
        frontleft.setPower(speed1); //set motor power target
        frontright.setPower(speed2);
        backleft.setPower(speed3);
        backright.setPower(speed4);

        //run motors until one of them stops
        while(parent.opModeIsActive() &&  (runtime.seconds() < timeout) && (frontleft.isBusy()
                && frontright.isBusy() && backleft.isBusy() && backright.isBusy())){

            telemetry.addData("encoder-fwd-left", frontleft.getCurrentPosition() + "busy=" + frontleft.isBusy());
            telemetry.addData("encoder-fwd-right", frontright.getCurrentPosition() + "busy=" + frontright.isBusy());
            telemetry.addData("encoder-bkw-left", backleft.getCurrentPosition() + "busy=" + backleft.isBusy());
            telemetry.addData("encoder-bkw-right", backright.getCurrentPosition() + "busy=" + backright.isBusy());
            telemetry.update();
        }

        stopMovement();
        runtime.reset();
    }

    public void stopMovement(){
        frontleft.setPower(0);
        frontright.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);

        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        try{
            Thread.sleep(100);
        }
        catch(Exception e){
            telemetry.addData("sleep failed", "");
        }
    }

    //MOVEMENT COMMANDS
    public void moveForward(double speed, int distance){
        setMotorPosition(-distance, -distance, -distance, -distance);
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        setMotorPower(speed, speed, speed, speed);
    }

    public void moveBackward(double speed, int distance){
        setMotorPosition(distance, distance, distance, distance);
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        setMotorPower(speed, speed, speed, speed);
    }

    public void moveLeft(double speed, int distance){
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontleft.setTargetPosition(frontleft.getCurrentPosition() + distance);
        frontright.setTargetPosition(frontright.getCurrentPosition() - distance);
        backleft.setTargetPosition(backleft.getCurrentPosition() + distance);
        backright.setTargetPosition(backright.getCurrentPosition() - distance);

        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontright.setPower(-speed);
        frontleft.setPower(speed);
        backright.setPower(speed);
        backleft.setPower(-speed);

        runtime.reset();
        stopMovement();
    }

    public void turnLeft(double speed, int distance){
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + distance);
        frontright.setTargetPosition(frontright.getCurrentPosition() - distance);
        backleft.setTargetPosition(backleft.getCurrentPosition() - distance);
        backright.setTargetPosition(backright.getCurrentPosition() + distance);

        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontright.setPower(speed);
        frontleft.setPower(speed);
        backright.setPower(speed);
        backleft.setPower(speed);

        runtime.reset();
        stopMovement();
    }

    public void moveRight(double speed, int distance){
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        frontleft.setTargetPosition(-distance);
        frontright.setTargetPosition(distance);
        backleft.setTargetPosition(distance);
        backright.setTargetPosition(-distance);

        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontright.setPower(speed);
        frontleft.setPower(-speed);
        backright.setPower(-speed);
        backleft.setPower(speed);

        runtime.reset();
        stopMovement();
    }

    public void turnRight(double speed, int distance){
        frontleft.setTargetPosition(frontleft.getCurrentPosition() - distance);
        frontright.setTargetPosition(frontright.getCurrentPosition() + distance);
        backleft.setTargetPosition(backleft.getCurrentPosition() + distance);
        backright.setTargetPosition(backright.getCurrentPosition() - distance);

        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontright.setPower(speed);
        frontleft.setPower(speed);
        backright.setPower(speed);
        backleft.setPower(speed);

        runtime.reset();
        stopMovement();
    }


    //ARM COMMANDS
    public void moveArm(int distance){
        arm.setTargetPosition(distance);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);
    }

}
