// length_robo_ms.cpp ver1.0
// 改善案があったら好きに書き換えてね。
// いやだね！！！byしゃか
// length = rad * (2π / 360)(0.01744) * rote

#include "rote_robo_ms.h"
#include "mbed.h"

void rote_robo_ms::length_to_rote_set(int length, int torque, bool brake, bool debugmsg)
{

    _length = length;
    if (brake)
    {
        ;
    }
    else if (torque > 0)
    {
        _torque_sign = 1;
        _torque = torque;
    }
    else
    {
        _torque_sign = -1;
        _torque = -torque;
    } // torqueはトルクの絶対値、signが符号
    _debugmsg = debugmsg;
    _brake = brake;
    sumRdynamic = 0;
    deltaR = 0;
    stop = false;
}
void rote_robo_ms::rote_robo_ms_update(CANMessage *msg, int BUFFER_MAX)
{
    // if(_debugmsg)printf("motor%d sumS:%d sumD:%d rote:%d spd:%d deltaR:%d tmpR:%d\n",_motornum,(int)sumRstatic,(int)sumRdynamic,rote,spd,deltaR,tmpR);
    int msgnum = 0x201 + _motornum;
    for (int i = 0; i < BUFFER_MAX; i++)
    {
        if (msg[i].id == msgnum)
            _msg = msg[i];
    }
    _rbms.rbms_read(_msg, &rote, &spd);
    
    if (rote < tmpR && spd > 0)
    {
        deltaR = (short)(360 - tmpR) + rote;
    }
    else if (rote > tmpR && spd > 0)
    {
        deltaR = rote - tmpR;
    } // 正転時
    if (rote > tmpR && spd < 0)
    {
        deltaR = (short)(360 - rote) + tmpR;
        deltaR *= -1;
    }
    else if (rote < tmpR && spd < 0)
    {
        deltaR = rote - tmpR;
    } // 反転時    
    if (rote == tmpR) // ごり押した部分!!!!
    {
        deltaR = 0;
    }

    position = (float)sumRstatic / 36 * 0.01744 * _rad;
    // printf("%x\n",msgnum);
    // printf("motor%d length:%d spd:%d rote:%d deltaR:%d\n",_motornum,(int)sumLength,spd,(int)sumRdynamic,deltaR);

    sumRdynamic += deltaR;
    sumRstatic += deltaR;

    sumR_main = (float)sumRdynamic / 36;
    tmpR = rote;
}
void rote_robo_ms::rote_robo_ms_update_write(int *motor, int submotornum)
{
    // printf("%d\n",deltaR);
    if (spd * _torque_sign <= 1500 && pulsebrake)
    { // 逆転信号を出した後速度が1500以下まで減速したら
        //printf("motor%d length:%d spd:%d rote:%d deltaR:%d position:%d\n", _motornum, (int)sumLength, spd, (int)sumRdynamic, deltaR, position);
        stop = true; // トルクを0にしてから速度が0になるまでの間のみ真
        _brake = false;
        pulsebrake = false;
        motor[_motornum] = 0;
        if (submotornum != -1)
            motor[submotornum] = 0;
        deltaR = 0;
        // printf("motor%d: sumRS:%d deltaR:%d tmpR:%d rote:%d spd:%d\n",_motornum,(int)sumRstatic,deltaR,tmpR,(int)rote,spd);
    }
    // 3.14*2/360=0.17444...
    sumLength = _rad * 0.01744 * sumR_main;
    if ((sumLength * _torque_sign >= (float)_length || _brake) && !stop)
    {
        pulsebrake = true;
        motor[_motornum] = _torque * -_torque_sign * 4;
        if (submotornum != -1)
            motor[submotornum] = _torque * _torque_sign * 4;
        // printf("s*t:%d pulse:%d",spd*_torque_sign,pulsebrake);
    }
    else if (sumLength * _torque_sign >= (float)_length * 0.5 && !stop)
    {
        motor[_motornum] = (float)(_torque * _torque_sign) * 0.5;
        if (submotornum != -1)
            motor[submotornum] = (float)(_torque * -_torque_sign) * 0.5;
    }
    else if (!stop)
    {
        motor[_motornum] = _torque * _torque_sign;
        if (submotornum != -1)
            motor[submotornum] = _torque * -_torque_sign;
    }
    if (spd == 0 && stop == true)
    {
        deltaR = 0;
    }
    // printf("motor%d: sumRS:%d deltaR:%d tmpR:%d rote:%d spd:%d\n",_motornum,(int)sumRstatic,deltaR,tmpR,(int)rote,spd);
}

long rote_robo_ms::get_rote_sum()
{
    return sumRstatic;
}

void rote_robo_ms::set_static_reset(int num)
{
    sumRstatic = num;
}
