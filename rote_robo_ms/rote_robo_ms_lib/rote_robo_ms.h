#ifndef ROTE_ROBO_MS_H
#define ROTE_ROBO_MS_H

#include "mbed.h"
#include "rbms.h"

class rote_robo_ms
{
public:
    rote_robo_ms(CAN &can, rbms rbms, int motornum, float rad = 16) // CANのインスタンス,rbmsのインスタンス,制御するモータの番号(0から),制御するモータの半径
        : _can(can), _rbms(rbms), _motornum(motornum), _rad(rad), sumRstatic(0)
    {
        _can.frequency(1000000); // CANのビットレートを指定
        _can.mode(CAN::Normal);  // CANのモードをNormalに設定
    }

    void length_to_rote_set(int length, int torque, bool brake = false, bool debugmsg = false);
    void rote_robo_ms_update(CANMessage *msg, int BUFFER_MAX);
    void rote_robo_ms_update_write(int *motor, int submotornum = -1);
    long get_rote_sum();
    void set_static_reset(int num);
    int position = 0;

private:
    CAN &_can;
    CANMessage _msg;
    rbms _rbms;
    bool pulsebrake = false, stop = false, _debugmsg, _brake;
    short deltaR, tmpR, rote, spd;
    int _rad, _motornum, _length, _torque, _torque_sign;
    long sumRdynamic, sumRstatic; // 動き出してからの角度の合計(停止でリセット)、起動してからの角度の合計(プログラム終了でリセット)
    float sumLength, sumR_main;
};

#endif
