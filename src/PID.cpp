#include "cmath"
#include "vex.h"
#include "motors.h"
#include "PID.h"

//using namespace _PID;
    
    PID::PID() {
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

        printToConsole("Position: " << position);
        // printToConsole("Left Side Turns: " << fabs(Tl.position(vex::turns)));
        // printToConsole("Right Side Turns: " << fabs(Tr.position(vex::turns)));

        if (error == prev && position >= 3) {
            printToConsole("The error is not changing. PID stopping.");
            Brain.Screen.print("The error is not changing. PID stopping.");
            stopPID();
            errorChanging = false;
            stop = true;
        } 

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

        
    }

    bool PID::isStopped() {
        if (((Left.velocity(vex::rpm) + Right.velocity(vex::rpm))/2 <= 1 && fabs(position) >= 3) || ((!errorChanging) && (Left.velocity(vex::rpm) + Right.velocity(vex::rpm))/2 <= 1) || stop) return true;
        else return false;
    }

    void PID::stopPID() {
        errorChanging = false;
        Left.stop(vex::brake);
        Right.stop(vex::brake);
        Left.setStopping(vex::coast);
        Right.setStopping(vex::coast);
        stop = true;
    }

    double PID::kpUpdate(double addAmount) {
        kp += addAmount;
        return kp;
    }

    double PID::kdUpdate(double addAmount) {
        kd += addAmount;
        return kd;
    }

    void PID::runPID(double targetVal, double timeLimit)
    {
        reset();
        target = targetVal;
        while (fabs(position - target) > 0.2 && errorChanging) {
            update();
            //spinAll(true, (kp * error) + (ki * i) + (kd * d));
            Left.spin(vex::forward, (kp * error) + (ki * i) + (kd * d), vex::pct);
            Right.spin(vex::forward, (kp * error) + (ki * i) + (kd * d), vex::pct);
            if (isStopped()) { stopPID(); break; }
            _time += 20;
            vex::wait(20, vex::msec);

            if (_time >= timeLimit * 1000) {
                printToConsole("Time limit reached");
                stopPID();
                break;
            }
        }

        vex::wait(20, vex::msec);
    }