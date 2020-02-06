/**
 * @file Sakura_modules.cpp
 * @brief GR-SAKURA 実装用クラス ( S_Fets , S_UnderBody ) メンバの実装
 */


#include "Sakura_modules.h"

S_Fets::S_Fets(HardwareSerial *_comm, char _id, Fets::portNum outputPort, Fets::portNum inputPort) : Fets(_id, outputPort, inputPort){
    comm = _comm;
}

void S_Fets::begin(int baudrate){
    comm -> begin(baudrate);
}

void S_Fets::send(char data){
    comm->write(data);
}

int S_Fets::recieve(){
    return comm->read();
}



S_UnderBody::S_UnderBody(HardwareSerial *_comm) : UnderBody(){
    comm = _comm;
}

void S_UnderBody::begin(int baudrate){
    comm->begin(baudrate);
}

void S_UnderBody::send(char data){
    comm->write(data);
}