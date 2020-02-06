/**
 * @file Fets.cpp
 * @brief Fets クラスメンバの実装
 */

#include "Fets.h"


char Fets::mode = MODE_INIT;

Fets::Fets(char _id, portNum outputPort, portNum inputPort){

    char newMode = MODE_INIT;

    if(outputPort == None) {       // ポートの指定がない時
        newMode = MODE_MODULE;
    }
    else {   // ポートの指定がある時
        newMode = MODE_PORT;
    }

    if(mode != MODE_INIT && newMode != mode) {  // mode 1 と mode 2,3 は両立不可
        mode = MODE_CONFLICT;
        opNum = None;
        ipNum = None;
    }
    else {
        mode = newMode;
        id = _id;

        opNum = outputPort;
        ipNum = inputPort;
    }
}


int Fets::write(int duty, portNum outputPort){
    if((outputPort = opCheck(outputPort)) == None) return -1;
    if(duty != 0 && duty != 1) return -2;

    return sendData(FUNC_DIGITAL_OUT, outputPort, duty & 0x01);
}

int Fets::write(double duty, portNum outputPort){
    if((outputPort = opCheck(outputPort)) == None) return -1;
    if(outputPort == Out7) return -2;
    if(duty < 0.0) return -3;
    if(duty > 1.0) return -4;

    return sendData(FUNC_PWM_OUT, outputPort, (uint8_t)(duty*127.0));
}

int Fets::sensorResponce(uint8_t actInput, uint8_t actOutput, portNum outputPort, portNum inputPort){
    if((outputPort = opCheck(outputPort)) == None) return -1;
    if((inputPort  = ipCheck(inputPort))  == None) return -2;

    sendData(FUNC_SENSOR_RES, outputPort, 0x7F & (((actInput & 0x01) << 4) | ((actOutput & 0x01) << 3) | ((inputPort - Dammy) & 0x07)));

    return ((outputState >> (outputPort - 1)) & 0x01) == actOutput;
}

int Fets::sensorTrigger(uint8_t actInput, uint8_t actOutput, portNum outputPort, portNum inputPort){
    if((outputPort = opCheck(outputPort)) == None) return -1;
    if((inputPort  = ipCheck(inputPort))  == None) return -2;

    sendData(FUNC_SENSOR_TRG, outputPort, 0x7F & (((actInput & 0x01) << 4) | ((actOutput & 0x01) << 3) | ((inputPort - Dammy) & 0x07)));

    return ((outputState >> (outputPort - 1)) & 0x01) == actOutput;
}

int Fets::writeWave(waveform form, int period, portNum outputPort){
    if((outputPort = opCheck(outputPort)) == None) return -1;
    if(outputPort == Out7)  return -2;
    if(period < 100)        return -3;
    if(period > 10000)      return -4;

    return sendData((uint8_t)form, outputPort, (uint8_t)(period/100));
}



int Fets::getOutputState(portNum outputPort){
    if(mode == MODE_CONFLICT) return -1;

    recvData();
    if((outputPort = opCheck(outputPort)) == None) return (int)outputState;

    return (outputState >> (outputPort - Out1)) & 0x01;
}

int Fets::getInputState(portNum inputPort){
    if(mode == MODE_CONFLICT) return -1;

    recvData();
    if((inputPort = ipCheck(inputPort)) == None) return (int)inputState;

    return (inputState >> (inputPort - In1)) & 0x01;
}

int Fets::sendData(uint8_t funcBit, portNum outputPort, uint8_t parameter){
    uint8_t str[4] = {};

    if((outputPort = opCheck(outputPort)) == None) return -1;

    str[0] = ((funcBit << 3) & 0x78) | (outputPort & 0x07);
    str[1] = parameter;
    str[2] = str[0] ^ str[1];
    str[3] = id;

    for(int i=0; i<4; i++){
        send(str[i]);
    }
    return 0;
}

int Fets::recvData(){
    if(mode == MODE_CONFLICT) return -1;

    int getNum = 0;
    int data;

    while((data = recieve()) != -1){

        dataBuff[3] = (char)data;

        if(dataBuff[3] == id
        && dataBuff[2] == (dataBuff[0] ^ dataBuff[1])){
            outputState = dataBuff[1];
            inputState  = dataBuff[0];
        }

        for(int i=0;i<3;i++){
            dataBuff[i] = dataBuff[i+1];
        }

        getNum++;
    }

    return getNum;
}


Fets::portNum Fets::ipCheck(portNum inputPort){

    if(mode == MODE_CONFLICT){
        return None;
    }
    else if(inputPort > Dammy && inputPort <= In7){
        return inputPort;
    }
    else if(ipNum > Dammy){
        return ipNum;
    }

    return None;
}

Fets::portNum Fets::opCheck(portNum outputPort){
    
    if(mode == MODE_CONFLICT){
        return None;
    }
    else if(outputPort > None && outputPort < Dammy){
        return outputPort;
    }
    else if(opNum > None && opNum < Dammy){
        return opNum;
    }

    return None;
}