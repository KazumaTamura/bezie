/**
 * @file UnderBody.cpp
 * @brief UnderBody クラスメンバの実装
 */


#include "UnderBody.h"


UnderBody::UnderBody(){

}

int UnderBody::moveXY(int vX, int vY, int omega){
    if(vX > MAX_VELO)   return -1;
    if(vX < -MAX_VELO)  return -2;
    if(vY > MAX_VELO)   return -3;
    if(vY < -MAX_VELO)  return -4;
    if(omega > MAX_OMEGA)   return -5;
    if(omega < -MAX_OMEGA)  return -6;

    sendData(vX, vY, omega, MOVE_RECT);
    return 0;
}

int UnderBody::moveXY(double vX, double vY, double omega){
    return moveXY((int)(vX*1000.0), (int)(vY*1000.0), (int)(omega*180.0/PI));
}

int UnderBody::movePolar(int spd, int dir, int omega){
    if(spd > MAX_VELO)  return -1;
    if(spd < -MAX_VELO) return -2;
    if(omega > MAX_OMEGA)   return -3;
    if(omega < -MAX_OMEGA)  return -4;

    while(dir > 360) dir -= 360;
    while(dir < 0)   dir += 360;

    sendData(spd, dir, omega, MOVE_POLAR);
    return 0;
}

int UnderBody::movePolar(double spd, double dir, double omega){
    return movePolar((int)(spd*1000.0), (int)(dir*180.0/PI), (int)(omega*180.0/PI));
}

void UnderBody::stop(){
    sendData(0, 0, 0, MOVE_STOP);
}

void UnderBody::sendData(int param1, int param2, int param3, uint8_t mode){

    uint8_t data[8] = {};

    if(param1 < 0.0){
        data[0] = 0x40;
        param1 *= -1;
    }    
    data[0] |= ((param1 >> 7) & 0x3F);
    data[1] = param1 & 0x7F;

    if(param2 < 0.0){
        data[2] = 0x40;
        param2 *= -1;
    }
    data[2] |= ((param2 >> 7) & 0x3F);
    data[3] = param2 & 0x7F;

    if(param3 < 0.0){
        data[4] = 0x40;
        param3 *= -1;
    }
    data[4] |= ((param3 >> 7) & 0x3F);
    data[5] = param3 & 0x7F;

    data[6] = data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5];
    data[7] = mode;

    for(int i=0; i<8; i++){
        send(data[i]);
    }
}