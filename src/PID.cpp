#include "cmath"
#include "vex.h"
#include "motors.h"
#include "PID.h"
#include "turnPID.h"
#include <fstream>

//using namespace _PID;
    
    PID::PID() {
    }

    void PID::setTurnPID(turnPID newTP) {
        tp = newTP;
    }

    void PID::reset()
    {
        error = 0;
        prev = 0;
        i = 0;
        Left.resetPosition();
        Right.resetPosition();
        _time = 0;
        position = 0;
        errorChanging = true;
        stop = false;
    }

    void PID::update()
    {
        position = ((fabs(Left.position(vex::turns)) + fabs(Right.position(vex::turns))) / 2.0) * M_PI * 3.25;
        error = target - position;

        //printToConsole(fabs((kp * error) + (ki * i) + (kd * d)));

        
        //printToConsole(/*"(kp * error) " <<*/ (kp * error));
        // printToConsole("(ki * i) " << (ki * i));
        // printToConsole("(kd * d) " << (kd * d));

        //printToConsole("Position: " << position);
        // printToConsole("Left Side Turns: " << fabs(Tl.position(vex::turns)));
        // printToConsole("Right Side Turns: " << fabs(Tr.position(vex::turns)));

        /*if (error == prev && position >= 3) {
            printToConsole("The error is not changing. PID stopping.");
            Brain.Screen.print("The error is not changing. PID stopping.");
            stopPID();
            errorChanging = false;
            stop = true;
        } */

        i = i + error + (prev - error) / 2.0;
        d = error - prev;
        prev = error;
        if (error == 0)
        {
            i = 0;
        }

        if (fabs(i) >= 100)
        {
            i = (i / fabs(i)) * 100;
        }

        if ((error / fabs(error)) != (prev / fabs(prev))) {
            stopPID();
            printToConsole("error turned negative");
        }
        
    }

    bool PID::isStopped() {
        //printToConsole(((Left.velocity(vex::rpm) + Right.velocity(vex::rpm))/2 <= 1));
        if (((fabs(Left.velocity(vex::rpm)) + fabs(Right.velocity(vex::rpm)))/2 <= 1 && fabs(position) >= 3) || ((!errorChanging) && (Left.velocity(vex::rpm) + Right.velocity(vex::rpm))/2 <= 1) || stop) return true;
        else return false;
    }

    void PID::stopPID() {
        errorChanging = false;
        Left.stop(vex::brake);
        Right.stop(vex::brake);
        Left.setStopping(vex::coast);
        Right.setStopping(vex::coast);
        stop = true;
        printToConsole("cringe");
    }

    // double PID::kpUpdate(double addAmount) {
    //     kp += addAmount;
    //     return kp;
    // }

    // double PID::kdUpdate(double addAmount) {
    //     kd += addAmount;
    //     return kd;
    // }

    void PID::runPID(double targetVal, double timeLimit)
    {
        reset();
        target = targetVal / (M_PI - 1);
        int negative = target / fabs(target);
        target = fabs(target);

        while (fabs(position - target) > 0.2 && errorChanging) {
            update();
            //spinAll(true, (kp * error) + (ki * i) + (kd * d));
            Left.spin(vex::forward, fabs((kp * (target - pow(position, 1.3))) + (ki * i) + (kd * d)) * (negative) * 1.7, vex::pct);
            Right.spin(vex::forward, fabs((kp * (target - pow(position, 1.3))) + (ki * i) + (kd * d)) * (negative) * 1.7, vex::pct);
            if (isStopped()) { stopPID(); break; }
            _time += 20;
            vex::wait(20, vex::msec);

            if (fabs((kp * error) + (ki * i) + (kd * d)) < 1) {
                printToConsole("Too slow; saving time");
                stopPID();
                break;
            }

            if (_time >= timeLimit * 1000) {
                printToConsole("Time limit reached");
                stopPID();
                break;
            }
        }
        printToConsole("time: " << _time);
        vex::wait(20, vex::msec);
    }

    void PID::shake(double seconds) {
        int time = 0;
        int timeLimit = seconds * 1000; //msec
        double startPos = InertialSensor.heading();
        bool isFwd = true;
        while (time <= timeLimit) {
            if (isFwd) {
                Left.spin(vex::forward, 100, vex::pct);
                Right.spin(vex::forward, 100, vex::pct);
            } else {
                Left.spin(vex::reverse, 100, vex::pct);
                Right.spin(vex::reverse, 100, vex::pct);
            }
            wait(140, vex::msec);
            Left.stop(vex::brake);
            Right.stop(vex::brake);
            Left.setStopping(vex::coast);
            Right.setStopping(vex::coast);

            isFwd = !isFwd;
            time += 140;
        }
        printToConsole("[PID Shake] Done shaking");
        stopPID();
        if ((InertialSensor.heading() + 3 <= startPos) || (InertialSensor.heading() - 3 >= startPos)) /*If not in range (3 in either direction) */ {
            tp.runTurnPID(startPos);
        }
    }