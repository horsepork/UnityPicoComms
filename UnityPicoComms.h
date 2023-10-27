#ifndef UNITYPICOCOMMS_H
#define UNITYPICOCOMMS_H

#include "Arduino.h"
#include "PacketSerial.h"
#include "FastLED.h"
#include "LibPrintf.h"
#include "PicoBuzzer.h"

#define TIME_BETWEEN_MESSAGES 20
#define LED_PIN 25

void onPacketReceived(const uint8_t* buffer, size_t size);
void FastLED_Update();

struct InputObjectStruct{
    public:
        InputObjectStruct(uint8_t _msgType, size_t _dataSize, void(*_callback)()){
            msgType = _msgType;
            dataSize = _dataSize;
            data = new uint8_t[dataSize];
            callback = _callback;
        }
        uint8_t msgType;
        uint8_t* data;
        size_t dataSize;
        void(*callback)() = nullptr;
};

struct NeopixelInputObjectStruct{
    public:
        NeopixelInputObjectStruct(uint8_t _msgType, CRGB* _data, size_t _numNeopixels){
            msgType = _msgType;
            data = _data;
            numNeopixels = _numNeopixels;
        }
        uint8_t msgType;
        CRGB* data;
        size_t numNeopixels;
};

struct OutputObjectStruct{
    public:
        OutputObjectStruct(uint8_t _msgType, size_t _dataSize, bool(*_callback)(bool)){
            msgType = _msgType;
            dataSize = _dataSize;
            data = new uint8_t[dataSize];
            callback = _callback;
        }
        uint8_t msgType;
        uint8_t* data;
        size_t dataSize;
        bool(*callback)(bool) = nullptr;
};

enum UnityPicoCommsPacketEnum{
    INVALID_PACKET = 0,
    ID_MSG = 0b00100000,
    CONFIRMATION_RESPONSE = 0b01000000,
    BIG_PACKET_MSG = 0b01100000,
    DATA_PACKET = 0b10000000,
    LARGE_DATA_PACKET = 0b10100000,
    ERROR_MSG = 0b11100000 // not implemented yet, add if useful for debugging
};

class OutputMessageObject{
    public:
        void activate(uint8_t _messageType, uint8_t* _buf, size_t _size, bool(*callback)(bool)){
            buf = _buf;
            size = _size;
            activated = true;
            messageType = _messageType; // bit redundant since it's also the outputObject array index, but whatevz™
            update = callback;
            for(int i = 0; i < size; i++){
                buf[i] = 0;
            }
        }

        uint8_t messageType;
        uint8_t* buf = nullptr;
        size_t size;
        bool(*update)(bool) = nullptr;
        uint32_t timer;
        uint16_t automaticResendTime = 1000;
        int16_t expectedMessageConfirmation = -2;
        bool updated = true;
        bool activated = false;
};

class InputMessageObject{
    public:
        uint8_t messageType;
        uint8_t* buf = nullptr;
        CRGB* Neopixel_buf = nullptr;
        size_t size;
        bool updated = false;
        bool awaitingBigPacket = false;
        void (*onUpdate)() = nullptr;
        bool activated = false;
        void activate(uint8_t _messageType, uint8_t* _buf, size_t _size, void(*callback)()){
            messageType = _messageType;
            buf = _buf;
            size = _size;
            onUpdate = callback;
            activated = true;
            for(int i = 0; i < size; i++){
                buf[i] = 0;
            }
        }
        void activate(uint8_t _messageType, CRGB* _Neopixel_buf, size_t _size){
            messageType = _messageType;
            Neopixel_buf = _Neopixel_buf;
            size = _size * 3 + 2; // to allow for two bytes offset index
            onUpdate = FastLED_Update;
            activated = true;
        }
        
        void update(const uint8_t *_buf, size_t _size){
            // this is a bit of a mess and will not work for multiple separate
            // FastLED instances on a single input object. Which probably is fine
            if(Neopixel_buf){
                int16_t offsetIndex = _buf[0] + (_buf[1] >> 8) + 2;
                for(int i = offsetIndex; (i+2) < offsetIndex + (_size-2); i+=3){
                    Neopixel_buf[(i-2)/3].setRGB(_buf[i - offsetIndex + 2], _buf[i+1 - offsetIndex + 2], _buf[i+2 - offsetIndex + 2]);
                }
            }
            else{
                for(int i = 0; i < min(size, _size); i++){
                    buf[i] = _buf[i];
                }
            }
            onUpdate();
        }

        void setCallback(void(*_callback)()){
            onUpdate = _callback;
        }
};

template<size_t receiveBufferSize = 6400> 
class UnityPicoComms{    
    private:
        const char* PicoID;
        uint32_t baudRate = 921600;
        OutputMessageObject outputObjects[32];
        OutputMessageObject *activeOutputObjects[32];
        uint8_t numActiveOutputObjects;
        bool connected = false;
        uint16_t watchdogTimerLength = 3000;
        Stream* SerialPort = &Serial;
        bool pauseForIDMsgOutput = false;
        uint32_t IDMsgTimer;
    
    public:
        PacketSerial_<COBS, 0, receiveBufferSize> packetSerial;
        InputMessageObject inputObjects[32];
        bool FastLED_Updated = false;

        void setSerialPort(Stream* port){
            SerialPort = port;
        }

        void begin(const char* _id){
            pinMode(LED_PIN, OUTPUT);
            digitalWrite(LED_PIN, HIGH);
            PicoID = _id;
            packetSerial.setStream(SerialPort);
            packetSerial.setPacketHandler(&onPacketReceived);
            const char* SerialName;
            if(SerialPort == &Serial){
                Serial.begin(baudRate);
                // while(!Serial); // can't think of a good reason not to cycle through updates in the meantime
            }
            else if(SerialPort == &Serial1){
                Serial1.begin(baudRate);
            }
            else if(SerialPort == &Serial2){
                Serial2.begin(baudRate);
            }
            digitalWrite(LED_PIN, LOW);
            connected = true;
            rp2040.wdt_begin(watchdogTimerLength);
            for(int i = 0; i < numActiveOutputObjects; i++){
                activeOutputObjects[i]->update(true);
            }
        }

        void begin(const char* _id, uint32_t _baudRate){
            baudRate = _baudRate;
            begin(_id);
        }

        void setWatchdogTimerLength(uint16_t length){
            // note: must be called prior to begin
            watchdogTimerLength = length;
        }

        bool isConnected(){ // will only return false if using Serial and there is no USB connection, no way of verifying connection otherwise
            if(SerialPort == &Serial){
                return Serial;
            }
            return true;
        }

        void update(){
            rp2040.wdt_reset();
            if(isConnected()){
                packetSerial.update(); // get incoming Unity packets
            }
            for(int i = 0; i < numActiveOutputObjects; i++){
                if(activeOutputObjects[i]->update(false)){ // update will only return true once, but updated will stay true until correct confirmation is received, hence the separate if statements
                    activeOutputObjects[i]->updated = true;
                }
                if(millis() - activeOutputObjects[i]->timer > activeOutputObjects[i]->automaticResendTime){
                    activeOutputObjects[i]->updated = true;
                }
                if(activeOutputObjects[i]->updated){
                    
                    if(millis() - activeOutputObjects[i]->timer < TIME_BETWEEN_MESSAGES) continue;
                    
                    UnityPicoCommsPacketEnum outputPacketType = activeOutputObjects[i]->size > 255 ? LARGE_DATA_PACKET : DATA_PACKET;
                    if(pauseForIDMsgOutput){
                        if(millis() - IDMsgTimer > 100){
                            pauseForIDMsgOutput = false;
                            
                        }
                    }
                    else{
                        sendPacket(activeOutputObjects[i]->messageType, activeOutputObjects[i]->buf, activeOutputObjects[i]->size, outputPacketType);
                        activeOutputObjects[i]->timer = millis();
                        activeOutputObjects[i]->automaticResendTime = 1000 + random(1000);
                    }
                }
            }
            if(FastLED_Updated){
                FastLED.show();
                FastLED_Updated = false;
            }
        }

        void dataUpdated(uint8_t messageType){
            outputObjects[messageType].updated = true;
        }

        void addOutput(uint8_t messageType, uint8_t* buf, size_t size, bool(*callback)(bool)){
            if(messageType > 31 || outputObjects[messageType].activated || inputObjects[messageType].activated){
                Error();
            }
            outputObjects[messageType].activate(messageType, buf, size, callback);
            activeOutputObjects[numActiveOutputObjects] = &outputObjects[messageType];
            numActiveOutputObjects++;
        }

        void addOutput(OutputObjectStruct packet){
            addOutput(packet.msgType, packet.data, packet.dataSize, packet.callback);
        }

        void addInput(uint8_t messageType, uint8_t* buf, size_t size, void (*callback)()){
            if(messageType > 31 || outputObjects[messageType].activated || inputObjects[messageType].activated){
                Error();
            }
            inputObjects[messageType].activate(messageType, buf, size, callback);
        }

        void addInput(InputObjectStruct packet){
            for(int i = 0; i < packet.dataSize; i++){
                packet.data[i] = 0;
            }
            addInput(packet.msgType, packet.data, packet.dataSize, packet.callback);
        }

        void addInput(uint8_t messageType, CRGB* buf, size_t size){
            if(messageType > 31 || outputObjects[messageType].activated || inputObjects[messageType].activated){
                Error();
            }
            inputObjects[messageType].activate(messageType, buf, size);
        }

        void addInput(NeopixelInputObjectStruct packet){
            addInput(packet.msgType, packet.data, packet.numNeopixels);
        }
        
        const char* getPicoID(){
            return PicoID;
        }

        void sendPacket(uint8_t messageType, char* str, size_t size, UnityPicoCommsPacketEnum packetType){
            sendPacket(messageType, (uint8_t*)str, size, packetType);
        }

        void sendPacket(uint8_t messageType, uint8_t value, UnityPicoCommsPacketEnum packetType){
            uint8_t buf[1] = {value};
            sendPacket(messageType, buf, 1, packetType);
        }

        void sendPacket(uint8_t messageType, uint8_t* buffer, size_t size, UnityPicoCommsPacketEnum packetType){
            if(!isConnected()) return;
            if(messageType > 31 || packetType == INVALID_PACKET){
                Error();
                return;
            }
            uint8_t dataStartOffset = 3;
            int16_t encodedSize = size + dataStartOffset; // start byte, checksum, size byte
            uint8_t startByte = (uint8_t)packetType + messageType;
            
            if(packetType == LARGE_DATA_PACKET){
                dataStartOffset = 4;
                encodedSize = size + dataStartOffset;
            }
            uint8_t outputBuffer[encodedSize];
            if(packetType == LARGE_DATA_PACKET){
                outputBuffer[3] = size >> 8;
            }

            uint8_t checkSum = 0;
            for(int i = 0; i < size; i++) checkSum += buffer[i];
            outputBuffer[0] = startByte;
            outputBuffer[1] = checkSum;
            outputBuffer[2] = size & 255;
            
            for(int i = dataStartOffset; i < encodedSize; i++){
                outputBuffer[i] = buffer[i-dataStartOffset];
            }
            packetSerial.send(outputBuffer, encodedSize);
            if((packetType == DATA_PACKET || packetType == LARGE_DATA_PACKET) && outputObjects[messageType].activated){
                outputObjects[messageType].expectedMessageConfirmation = checkSum;
                outputObjects[messageType].timer = millis();
            }
        }

        void verifyConfirmationMsg(uint8_t msgType, uint8_t incomingCheckSum){

            if(outputObjects[msgType].activated == false){
                return;
            }
            
            if(outputObjects[msgType].expectedMessageConfirmation == incomingCheckSum){
                
                outputObjects[msgType].expectedMessageConfirmation = -2;
                outputObjects[msgType].updated = false;
            }
        }

        void sendPicoID(){
            
            uint8_t ID_Length = strlen(PicoID);
            uint8_t ID_Buf[ID_Length];
            for(int i = 0; i < ID_Length; i++){
                ID_Buf[i] = PicoID[i];
            }
            sendPacket(0, ID_Buf, ID_Length, ID_MSG);
            Serial.println("Sending ID Msg");
            pauseForIDMsgOutput = true;
            IDMsgTimer = millis();
        }

        void resendOutputPackets(){
            for(int i = 0; i < numActiveOutputObjects; i++){
                activeOutputObjects[i]->updated = true;
            }
        }
 

        void sendBigPacketHandshake(uint8_t msgType){
            if(inputObjects[msgType].activated == false){
                return;
            }
            uint8_t bigPacketHandshakeBuf[1] = {0};
            sendPacket(msgType, bigPacketHandshakeBuf, 1, BIG_PACKET_MSG);
            inputObjects[msgType].awaitingBigPacket = true;
            uint32_t timer = millis();
            while(millis() - timer < 40){
                packetSerial.update();
                if(!inputObjects[msgType].awaitingBigPacket) break;
            }
        }

        void sendConfirmationResponse(uint8_t msgType, uint8_t checkSum){
            uint8_t confirmationBuf[1] = {checkSum};
            sendPacket(msgType, confirmationBuf, 1, CONFIRMATION_RESPONSE);
        }

    private:
        void Error(){
            while(true){
                digitalWrite(25, HIGH);
                delay(500);
                digitalWrite(25, LOW);
                delay(500);
            }
            
        }
};

UnityPicoComms comms;

void onPacketReceived(const uint8_t* buffer, size_t size) {
    uint8_t checkSum = buffer[1];
    size_t processedPacketSize = buffer[2];
    uint8_t dataStartIndex = 3;
    UnityPicoCommsPacketEnum packetType = (UnityPicoCommsPacketEnum)(0b11100000 & buffer[0]);
    if(packetType == LARGE_DATA_PACKET){ // if processedPacketSize > 255
        processedPacketSize += buffer[3] << 8;
        dataStartIndex = 4;
    }
    
    if(size < processedPacketSize + dataStartIndex){
        return;
    }

    uint8_t _checkSum = 0;
    for(int i = dataStartIndex; i < size; i++)_checkSum += buffer[i];
    if(_checkSum != checkSum){
        return;
    }
        
    uint8_t msgType = 0b00011111 & buffer[0];
    
    switch(packetType){
        case ID_MSG:
            comms.sendPicoID();
            comms.resendOutputPackets(); // so when Unity reconnects it gets the most current state from Pico
            break;
        case CONFIRMATION_RESPONSE:
            comms.verifyConfirmationMsg(msgType, buffer[3]);
            break;
        case BIG_PACKET_MSG:
            comms.sendBigPacketHandshake(msgType);
            break;
        case LARGE_DATA_PACKET:
        case DATA_PACKET:
            if(comms.inputObjects[msgType].activated == false){
                return;
            }
            if(processedPacketSize > comms.inputObjects[msgType].size){
                return; // error msg appropriate here
            }
            comms.sendConfirmationResponse(msgType, checkSum);
            comms.inputObjects[msgType].update(buffer + dataStartIndex, processedPacketSize);
            comms.inputObjects[msgType].awaitingBigPacket = false;
            break;
    }
}

 // does not contain "FastLED.show() because we only want to call it once
 // when we have multiple FastLED instances
void FastLED_Update(){
    comms.FastLED_Updated = true;
}

uint16_t power(uint8_t base, uint8_t exponent){ // not used here, but making it accessible for all Picos that may need it
    uint16_t returnVal = 1;
    for(int i = 0; i < exponent; i++){
        returnVal *= base;
    }
    return returnVal;
}

#endif