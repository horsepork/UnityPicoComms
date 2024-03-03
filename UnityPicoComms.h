#ifndef UNITYPICOCOMMS_H
#define UNITYPICOCOMMS_H

#include "Arduino.h"
#include "LibPrintf.h"
#include "PicoBuzzer.h"
#include "PacketSerial_Modified.h"

// output objects (objects, like the actual buttons, not packets) should probably always update their states.
// This removes the need for "forceUpdate" in the output object callback
// for context, a 72 byte array would take between 2 and 4 microseconds to update at 120MHz (assuming 4 or 5 cpu cycles per byte set)
// Even if I'm off by an order of magnitude, speed is probably not going to be a concern

#define TIME_BETWEEN_REATTEMPTED_MESSAGE 50 // give the other side time to process
#define TIME_BETWEEN_FULL_PACKET_RESEND 5000 // plus some fuzzing. TODO -- determine if needed
#define LED_PIN 25

#define ID_REQUEST_MESSAGE 0b00001111
#define MESSAGE_TYPE_MASK 0b00001111
#define PICO_DESIGNATOR_MASK 0b01110000 // value used if Pico is not connected through a hub
#define DEBUG_MESSAGE 0b10000000 
#define SMALL_INDEX_MASK  0b01111111
#define PING_MASK 0b10001111 // note that ping is a combo of debug and id request, so need to make sure that those are not errantly processed on a ping

#define FIFO_SIZE 2048
#define MAX_NUM_INPUT_AND_OUTPUT_OBJECTS 15

uint8_t IncomingPacket[FIFO_SIZE]; // after cobs decoding, before processing (has config byte, checksum, and indices)
uint8_t OutgoingPacket[FIFO_SIZE]; // before cobs encoding and sending

// TODO ---------
// make sure on connection that the update function for the output packet is called before any attempts to send

struct PacketHeader{
    uint8_t MessageType; // MessageType and PicoDesignatorCode make up Config byte at Index 0
    uint8_t PicoDesignatorCode;
    uint8_t Checksum; // Index 1. When outgoing, checksum is of (potentially) partial packet, when incoming, full packet

    // if either index exceeds 127, the 8th bit indicates a second byte is needed. Second byte is shifted left by 7
    uint16_t FirstIndex; // Index 2 and potentially 3
    uint16_t LastIndex; // Index 3 (or 4) and potentially 4 (or 5)

    // not actually part of the packet header, just useful
    int DataStartOffset;
    int DataSize;
};

PacketHeader InputPacketHeader;

class OutputPacket{ // Equivalent to PicoOutputPacket. Will have a corresponding PicoInputPacket on the Unity side
    public:
        OutputPacket(const char* _name, uint8_t _messageType, size_t _outputBufferSize, void(*_update)(OutputPacket&)){
            name = _name;
            messageType = _messageType;
            outputBufferSize = _outputBufferSize;
            update = _update;
            IndexOfLastChangedBufferElement = outputBufferSize;
            outputBuffer = new uint8_t[outputBufferSize];
            for(int i = 0; i < outputBufferSize; i++){
                outputBuffer[i] = 0;
            }
        }
        const char* name = nullptr;
        uint8_t messageType;
        size_t outputBufferSize;
        void(*update)(OutputPacket&) = nullptr;
        uint8_t* outputBuffer = nullptr;

        uint16_t IndexOfFirstChangedBufferElement = 0;
        uint16_t IndexOfLastChangedBufferElement;
        bool doUpdatesNeedToBeSent = false;
        uint32_t sendPacketReattemptTimer;
        uint32_t fullPacketResendTimer; // TODO -- determine if necessary
        uint16_t timeBeforeFullPacketResend; // TODO -- determine if necessary
        uint8_t outputBufferChecksum;

        // decide if necessary or helpful to have buffer update function which can receive an array
        // currently updates are down a byte at a time, with one function call per byte

        void updateBufferAtByte(int byteIndex, uint8_t newByte)
        {
            if(byteIndex >= outputBufferSize){
                printf("Error. ByteIndex %i is too large for %s. Max size is %i\n", byteIndex, name, outputBufferSize);
                return;
            }
            if (outputBuffer[byteIndex] != newByte)
            {
                outputBuffer[byteIndex] = newByte;
                doUpdatesNeedToBeSent = true;
                sendPacketReattemptTimer = millis() - TIME_BETWEEN_REATTEMPTED_MESSAGE; // so any new updates will send right away. TODO -- verify this is not bad
                updateMinAndMaxBufferIndices(byteIndex);
            }
        }

        void updateBufferAtByteAndBit(int byteIndex, uint8_t bitIndex, bool bitState){
            if(byteIndex >= outputBufferSize){
                printf("Error. ByteIndex %i is too large for %s. Max size is %i\n", byteIndex, name, outputBufferSize);
                return;
            }
            if(bitIndex > 7){
                printf("Error. Bit index of %i passed to %s\n", bitIndex, name);
                return;
            }
            uint8_t prevByteState = outputBuffer[byteIndex];
            bitWrite(outputBuffer[byteIndex], bitIndex, bitState);
            if(outputBuffer[byteIndex] != prevByteState){
                doUpdatesNeedToBeSent = true;
                sendPacketReattemptTimer = millis() - TIME_BETWEEN_REATTEMPTED_MESSAGE; // so any new updates will send right away. TODO -- verify this is not bad
                updateMinAndMaxBufferIndices(byteIndex);
            }
        }

        void updateMinAndMaxBufferIndices(int index)
        {
            IndexOfFirstChangedBufferElement = min(IndexOfFirstChangedBufferElement, index);
            IndexOfLastChangedBufferElement = max(IndexOfLastChangedBufferElement, index);
        }

        void resetMinAndMaxBufferIndices()
        {
            IndexOfFirstChangedBufferElement = outputBufferSize;
            IndexOfLastChangedBufferElement = 0;
        }

        void makeBufferIndicesCoverFullPacket()
        {
            IndexOfFirstChangedBufferElement = 0;
            IndexOfLastChangedBufferElement = outputBufferSize;
        }

        void verifyChecksum()
        {
            if(InputPacketHeader.Checksum == outputBufferChecksum)
            {
                doUpdatesNeedToBeSent = false;
                resetMinAndMaxBufferIndices();
                timeBeforeFullPacketResend = 5000 + random(1000);
                fullPacketResendTimer = millis();
            }
            else
            {
                doUpdatesNeedToBeSent = true;
                makeBufferIndicesCoverFullPacket(); // so that we resend the entire packet
            }
        }
 
        bool isTimeToResendFullPacket(){
            return millis() - sendPacketReattemptTimer  >= TIME_BETWEEN_FULL_PACKET_RESEND;
        }
};

class InputPacket{ // Equivalent to PicoInputPacket. Will have a corresponding PicoOutputPacket on the Unity side
    public:
        const char* name;
        uint8_t messageType;
        size_t inputBufferSize;
        void (*onUpdate)(uint8_t*, size_t) = nullptr;
        uint8_t* inputBuffer = nullptr;
        bool updated = false;
        uint8_t inputBufferChecksum = 0;
        InputPacket(const char* _name, uint8_t _messageType, size_t _inputBufferSize, void(*_onUpdate)(uint8_t*, size_t)){
            name = _name;
            messageType = _messageType;
            inputBufferSize = _inputBufferSize;
            onUpdate = _onUpdate;
            inputBuffer = new uint8_t[inputBufferSize];
            inputBufferSize = _inputBufferSize;
            onUpdate = _onUpdate;
            for(int i = 0; i < inputBufferSize; i++){
                inputBuffer[i] = 0;
            }
        }

        // there could be room to further optimize by tracking the indices that were updated
        // could be helpful so that neopixel values only need to be set if the inputBuffer changed
        bool updateInputBuffer()
        {

            if (InputPacketHeader.LastIndex >= inputBufferSize)
            {
                printf("Error: last index exceeds bounds of inputBuffer on %s. Incoming last index is %i but inputBuffer size is %i\n", name, InputPacketHeader.LastIndex, inputBufferSize);
                return false;
            }
            bool valuesChanged = false;
            for (int i = InputPacketHeader.FirstIndex; i < InputPacketHeader.LastIndex; i++)
            {
                if (inputBuffer[i] != IncomingPacket[i + InputPacketHeader.DataStartOffset])
                {
                    inputBuffer[i] = IncomingPacket[i + InputPacketHeader.DataStartOffset];
                    valuesChanged = true;
                }
            }
            if (valuesChanged)
            {
                onUpdate(inputBuffer, inputBufferSize);
                inputBufferChecksum = 0; 
                for(int i = 0; i < inputBufferSize; i++){
                    inputBufferChecksum += inputBuffer[i];
                }
            }
            return true;
        }
};

// consider some sort of isConnected check if we haven't heard anything for awhile. Maybe a ping going back the other way?

class UnityPicoComms{    
    private:
        const char* PicoID;
        uint8_t picoDesignatorCode = 0;
        uint32_t baudRate = 921600;
        uint16_t watchdogTimerLength = 3000;
        HardwareSerial* SerialPort = &Serial1;

        InputPacket** inputPackets = new InputPacket*[MAX_NUM_INPUT_AND_OUTPUT_OBJECTS];
        OutputPacket** outputPackets = new OutputPacket*[MAX_NUM_INPUT_AND_OUTPUT_OBJECTS];
        uint8_t numInputPackets = 0;
        uint8_t numOutputPackets = 0;
        
    
    public:

        UnityPicoComms(const char* _PicoID){
            PicoID = _PicoID;
            for(int i = 0; i < numInputPackets; i++){
                inputPackets[i] = nullptr;
            }
            for(int i = 0; i < numOutputPackets; i++){
                outputPackets[i] = nullptr;
            }
        }

        void setSerialPort(HardwareSerial* port){
            SerialPort = port;
        }

        void begin(){
            pinMode(LED_PIN, OUTPUT);
            SerialPort->begin(baudRate);
            rp2040.wdt_begin(watchdogTimerLength);
            for(int i = 0; i < numOutputPackets; i++){
                outputPackets[i]->update(*outputPackets[i]);
            }
        }

        void begin(uint32_t _baudRate){
            baudRate = _baudRate;
            begin();
        }

        void setWatchdogTimerLength(uint16_t length){
            // note: must be called prior to begin
            watchdogTimerLength = length;
        }



        void update(){
            rp2040.wdt_reset();

            handleIncomingPackets();
            handleOutputPackets();
            
        }

        void handleIncomingPackets(){
            uint32_t timeoutTimer = millis();
            while(millis() - timeoutTimer < 50){
                if(SerialPort->peek() == -1){
                    return;
                }
                int incomingPacketSize = _decodeBufferAndReturnSize(SerialPort, IncomingPacket);

                if(incomingPacketSize == 0){
                    continue;
                }
                if(!ValidateIncomingPacket()){
                    continue;
                }

                uint8_t _designatorCode = IncomingPacket[0] & PICO_DESIGNATOR_MASK;
                if(picoDesignatorCode != _designatorCode){
                    picoDesignatorCode = _designatorCode;
                    uint8_t readableDesignatorCode = picoDesignatorCode >> 4;
                    printf("Pico designator code: %i\n", readableDesignatorCode);
                }

                ProcessIncomingPacket(incomingPacketSize);
            }
        }

        bool ValidateIncomingPacket(){
            int offsetIndex = 0;
            const int defaultHeaderSize = 4;
            InputPacketHeader.Checksum = IncomingPacket[1];
            InputPacketHeader.FirstIndex = IncomingPacket[2] & SMALL_INDEX_MASK;
            if (IncomingPacket[2] > SMALL_INDEX_MASK)
            {
                InputPacketHeader.FirstIndex += (IncomingPacket[3] << 7);
                offsetIndex++;
            }
            InputPacketHeader.LastIndex = IncomingPacket[3 + offsetIndex] & SMALL_INDEX_MASK;
            if (IncomingPacket[3 + offsetIndex] > SMALL_INDEX_MASK)
            {
                InputPacketHeader.LastIndex += (IncomingPacket[3 + offsetIndex] << 7);
                offsetIndex++;
            }

            if (InputPacketHeader.FirstIndex > InputPacketHeader.LastIndex) { return false; }
            InputPacketHeader.DataStartOffset = defaultHeaderSize + offsetIndex;

            InputPacketHeader.DataSize = InputPacketHeader.LastIndex - InputPacketHeader.FirstIndex + 1;
            byte _checkSum = 0;
            for (int i = InputPacketHeader.DataStartOffset; i < InputPacketHeader.DataStartOffset + InputPacketHeader.DataSize; i++)
            {
                _checkSum += IncomingPacket[i];
            }

            if (_checkSum != InputPacketHeader.Checksum) { return false; }

            InputPacketHeader.MessageType = IncomingPacket[0] & MESSAGE_TYPE_MASK;
            InputPacketHeader.PicoDesignatorCode = IncomingPacket[0] & PICO_DESIGNATOR_MASK;

            return true;
        }


        void ProcessIncomingPacket(int incomingPacketSize)
        {
            
            if ((IncomingPacket[0] & PING_MASK) == PING_MASK)
            {
                respondToPing();
                return;
            }
            if (InputPacketHeader.MessageType == ID_REQUEST_MESSAGE)
            {
                sendPicoID();
                return;
            }
            
            for(int i = 0; i < numInputPackets; i++){
                if(inputPackets[i]->messageType == InputPacketHeader.MessageType){
                    if(inputPackets[i]->updateInputBuffer()){
                        SendPacket(inputPackets[i]->messageType, picoDesignatorCode, inputPackets[i]->inputBufferChecksum);
                    }
                    return;
                }
            }
            
            for(int i = 0; i < numOutputPackets; i++){
                if(outputPackets[i]->messageType == InputPacketHeader.MessageType){
                    outputPackets[i]->verifyChecksum();
                    return;
                }
            }
        }

        void handleOutputPackets(){
            for(int i = 0; i < numOutputPackets; i++){
                outputPackets[i]->update(*outputPackets[i]);
                // --- Comment out if we don't want to occasionally resend full pacekts ---
                if(outputPackets[i]->isTimeToResendFullPacket()){
                    outputPackets[i]->doUpdatesNeedToBeSent = true;
                    outputPackets[i]->makeBufferIndicesCoverFullPacket();
                }
                // ------------------------------------------------------------------------

                if(outputPackets[i]->doUpdatesNeedToBeSent){
                    
                    if(millis() - outputPackets[i]->sendPacketReattemptTimer < TIME_BETWEEN_REATTEMPTED_MESSAGE) continue;
                    
                    SendPacket(
                        outputPackets[i]->messageType,
                        picoDesignatorCode,
                        outputPackets[i]->IndexOfFirstChangedBufferElement,
                        outputPackets[i]->IndexOfLastChangedBufferElement,
                        outputPackets[i]->outputBuffer
                    );

                    outputPackets[i]->sendPacketReattemptTimer = millis();
                }
            }
        }

        void respondToPing(){
            SendPacket(PING_MASK + picoDesignatorCode, 0); // 0 because no data, just resonding to ping
        }


        void addOutputPacket(const char* _name, uint8_t _messageType, size_t _size, void(*_update)(OutputPacket &outputPacket)){
            if(_messageType >= MAX_NUM_INPUT_AND_OUTPUT_OBJECTS){
                Error();
            }
            for(int i = 0; i < numInputPackets; i++){
                if(inputPackets[i] != nullptr){
                    if(inputPackets[i]->messageType == _messageType){
                        printf("Error. Input packet %s already has messageType %i\n", inputPackets[i]->name, _messageType);
                        Error();
                    }
                }
            }
            for(int i = 0; i < numOutputPackets; i++){
                if(outputPackets[i] != nullptr){
                    if(outputPackets[i]->messageType == _messageType){
                        printf("Error. Output packet %s already has messageType %i\n", outputPackets[i]->name, _messageType);
                        Error();
                    }
                }
                OutputPacket newOutputPacket(_name, _messageType, _size, _update);
                outputPackets[numOutputPackets] = &newOutputPacket;
                break;
            }
            numOutputPackets++;
        }

        void addInputPacket(const char* _name, uint8_t _messageType, size_t _size, void(*callback)(uint8_t*, size_t)){
            if(_messageType >= MAX_NUM_INPUT_AND_OUTPUT_OBJECTS){
                Error();
            }
            for(int i = 0; i < numInputPackets; i++){
                if(inputPackets[i] != nullptr){
                    if(inputPackets[i]->messageType == _messageType){
                        printf("Error. Input packet %s already has messageType %i\n", inputPackets[i]->name, _messageType);
                        Error();
                    }
                }
            }
            for(int i = 0; i < numOutputPackets; i++){
                if(outputPackets[i] != nullptr){
                    if(outputPackets[i]->messageType == _messageType){
                        printf("Error. Output packet %s already has messageType %i\n", outputPackets[i]->name, _messageType);
                        Error();
                    }
                }
                InputPacket newInputPacket(_name, _messageType, _size, callback);
            }
            numInputPackets++;
        }

        const char* getPicoID(){
            return PicoID;
        }

        void SendPacket(byte configByte, int firstIndex, int lastIndex, uint8_t* buffer)
        {
            // prepare outgoing packet
            OutgoingPacket[0] = configByte;
            OutgoingPacket[2] = firstIndex & SMALL_INDEX_MASK;
            int dataOffset = 0;
            const int defaultHeaderSize = 4;
            if (firstIndex > SMALL_INDEX_MASK)
            {
                OutgoingPacket[3] = firstIndex >> 7;
                dataOffset++;
            }
            OutgoingPacket[3 + dataOffset] = firstIndex & SMALL_INDEX_MASK;
            if (lastIndex > SMALL_INDEX_MASK)
            {
                OutgoingPacket[4 + dataOffset] = lastIndex >> 7;
                dataOffset++;
            }
            byte checksum = 0;
            for (int i = firstIndex; i <= lastIndex; i++)
            {
                OutgoingPacket[dataOffset + defaultHeaderSize + i] = buffer[i];
                checksum += buffer[i];
            }
            OutgoingPacket[1] = checksum;

            int packetSize = defaultHeaderSize + dataOffset + lastIndex - firstIndex + 1;

            _packetSerialSend(SerialPort, OutgoingPacket, packetSize);
        }

        void SendPacket(uint8_t _messageType, uint8_t _picoDesignator, int firstIndex, int lastIndex, uint8_t* buffer){
            SendPacket(_messageType + _picoDesignator, firstIndex, lastIndex, buffer);
        }

        void SendPacket(uint8_t configByte, uint8_t singleOutputByte){
            uint8_t singleByteBuffer[1] = {singleOutputByte};
            SendPacket(configByte, 0, 0, singleByteBuffer);
        }

        void SendPacket(uint8_t _messageType, uint8_t _picoDesignator, uint8_t singleOutputByte){
            SendPacket(_messageType + _picoDesignator, singleOutputByte);
        }

        void sendPicoID(){
            uint8_t ID_Length = strlen(PicoID);
            uint8_t ID_Buf[ID_Length];
            for(int i = 0; i < ID_Length; i++){
                ID_Buf[i] = PicoID[i];
            }
            SendPacket(ID_REQUEST_MESSAGE + picoDesignatorCode, 0, ID_Length, ID_Buf);
        }

        void Debug(const char* debugString){
            uint8_t debugStringByteArray[strlen(debugString)];
            for(int i = 0; i < strlen(debugString); i++){
                debugStringByteArray[i] = (uint8_t)debugString[i];
            }
            SendPacket(DEBUG_MESSAGE + picoDesignatorCode, 0, strlen(debugString), debugStringByteArray);
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

uint16_t power(uint8_t base, uint8_t exponent){ // not used here, but making it accessible for all Picos that may need it
    uint16_t returnVal = 1;
    for(int i = 0; i < exponent; i++){
        returnVal *= base;
    }
    return returnVal;
}

#endif