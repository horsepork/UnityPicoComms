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
#define PICO_DESIGNATOR_MASK 0b01110000
#define DEBUG_MESSAGE 0b10000000 
#define SMALL_INDEX_MASK  0b01111111
#define PING_MASK 0b10001111 // note that ping is a combo of debug and id request, so need to make sure that those are not errantly processed on a ping

#define FIFO_SIZE 2048
#define MAX_NUM_INPUT_AND_OUTPUT_OBJECTS 15

uint8_t IncomingPacket[FIFO_SIZE]; // after cobs decoding, before processing (has config byte, checksum, and indices)
uint8_t OutgoingPacket[FIFO_SIZE]; // before cobs encoding and sending

// TODO ---------
// make sure on connection that the update function for the output packet is called before any attempts to send

struct {
    uint8_t MessageType; // MessageType and PicoDesignatorCode make up Config byte at Index 0
    uint8_t PicoDesignatorCode;
    uint8_t Checksum; // Index 1. When outgoing, checksum is of (potentially) partial packet, when incoming, full packet

    // if either index exceeds 127, the 8th bit indicates a second byte is needed. Second byte is shifted left by 7
    uint16_t FirstIndex; // Index 2 and potentially 3
    uint16_t LastIndex; // Index 3 (or 4) and potentially 4 (or 5)

    // not actually part of the packet header, just useful
    int DataStartOffset;
    int DataSize;
} PacketHeader;


class OutputPacket{ // Equivalent to PicoOutputPacket. Will have a corresponding PicoInputPacket on the Unity side
    public:        
        
        bool initialized = false;
        const char* name = nullptr;
        uint8_t messageType;
        size_t outputBufferSize;
        void(*update)(OutputPacket&) = nullptr;
        uint8_t* outputBuffer = nullptr;

        uint16_t IndexOfFirstChangedBufferElement = 0;
        uint16_t IndexOfLastChangedBufferElement;
        bool doUpdatesNeedToBeSent = true;
        uint32_t sendPacketReattemptTimer;
        uint32_t fullPacketResendTimer; // TODO -- determine if necessary
        uint16_t timeBeforeFullPacketResend = 5000; // TODO -- determine if necessary
        uint8_t outputBufferChecksum;

        // decide if necessary or helpful to have buffer update function which can receive an array
        // currently updates are down a byte at a time, with one function call per byte

        void initialize(const char* _name, uint8_t _messageType, size_t _outputBufferSize, void(*_update)(OutputPacket&)){
            name = _name;
            messageType = _messageType;
            outputBufferSize = _outputBufferSize;
            update = _update;
            IndexOfLastChangedBufferElement = outputBufferSize - 1;
            outputBuffer = new uint8_t[outputBufferSize];
            for(int i = 0; i < outputBufferSize; i++){
                outputBuffer[i] = 0;
            }
            initialized = true;
        }

        void updateBufferAtByte(int byteIndex, uint8_t newByte)
        {
            if(byteIndex >= outputBufferSize){
                // printf("Error. ByteIndex %i is too large for %s. Max size is %i\n", byteIndex, name, outputBufferSize);
                return;
            }
            if (outputBuffer[byteIndex] != newByte)
            {
                outputBuffer[byteIndex] = newByte;
                doUpdatesNeedToBeSent = true;
                setOutputBufferChecksum();
                for(int i = 0; i < outputBufferSize; i++){

                }
                sendPacketReattemptTimer = millis() - TIME_BETWEEN_REATTEMPTED_MESSAGE; // so any new updates will send right away. TODO -- verify this is not bad
                updateMinAndMaxBufferIndices(byteIndex);
            }
        }

        void updateBufferAtByteAndBit(int byteIndex, uint8_t bitIndex, bool bitState){
            if(byteIndex >= outputBufferSize){
                // printf("Error. ByteIndex %i is too large for %s. Max size is %i\n", byteIndex, name, outputBufferSize);
                return;
            }
            if(bitIndex > 7){
                // printf("Error. Bit index of %i passed to %s\n", bitIndex, name);
                return;
            }
            uint8_t prevByteState = outputBuffer[byteIndex];
            bitWrite(outputBuffer[byteIndex], bitIndex, bitState);
            if(outputBuffer[byteIndex] != prevByteState){
                setOutputBufferChecksum();
                doUpdatesNeedToBeSent = true;
                sendPacketReattemptTimer = millis() - TIME_BETWEEN_REATTEMPTED_MESSAGE; // so any new updates will send right away. TODO -- verify this is not bad
                updateMinAndMaxBufferIndices(byteIndex);
            }
        }

        void setOutputBufferChecksum(){ // the expected value to be received
            outputBufferChecksum = 0;
            for(int i = 0; i < outputBufferSize; i++){
                outputBufferChecksum += outputBuffer[i];
            }
        }

        void updateMinAndMaxBufferIndices(int index)
        {
            IndexOfFirstChangedBufferElement = min(IndexOfFirstChangedBufferElement, index);
            IndexOfLastChangedBufferElement = max(IndexOfLastChangedBufferElement, index);
        }

        void resetMinAndMaxBufferIndices()
        {
            IndexOfFirstChangedBufferElement = outputBufferSize - 1;
            IndexOfLastChangedBufferElement = 0;
        }

        void makeBufferIndicesCoverFullPacket()
        {
            IndexOfFirstChangedBufferElement = 0;
            IndexOfLastChangedBufferElement = outputBufferSize - 1;
        }

        void verifyChecksum()
        {
            if(PacketHeader.Checksum == outputBufferChecksum)
            {
                doUpdatesNeedToBeSent = false;
                resetMinAndMaxBufferIndices();
                timeBeforeFullPacketResend = 5000 + random(1000);
                fullPacketResendTimer = millis();
                // printf("valid checksum for %s\n", name);
            }
            else
            {
                // printf("checksum mismatch. Got %i but expected %i\n", PacketHeader.Checksum, outputBufferChecksum);
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
        bool initialized = false;
        void initialize(const char* _name, uint8_t _messageType, size_t _inputBufferSize, void(*_onUpdate)(uint8_t*, size_t)){
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
            initialized = true;
        }
        
        // there could be room to further optimize by tracking the indices that were updated
        // could be helpful so that neopixel values only need to be set if the inputBuffer changed
        bool updateInputBuffer()
        {
            if (PacketHeader.LastIndex >= inputBufferSize)
            {
                // printf("Error: last index exceeds bounds of inputBuffer on %s. Incoming last index is %i but inputBuffer size is %i\n", name, PacketHeader.LastIndex, inputBufferSize);
                return false;
            }
            bool valuesChanged = false;
            for (int i = PacketHeader.FirstIndex; i <= PacketHeader.LastIndex; i++)
            {
                if (inputBuffer[i] != IncomingPacket[i - PacketHeader.FirstIndex + PacketHeader.DataStartOffset])
                {
                    inputBuffer[i] = IncomingPacket[i - PacketHeader.FirstIndex + PacketHeader.DataStartOffset];
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
                // printf("new input buffer checksum: %i\n", inputBufferChecksum);
;            }
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
        SerialUART* SerialPort = &Serial1;
        InputPacket inputPackets[MAX_NUM_INPUT_AND_OUTPUT_OBJECTS];
        OutputPacket outputPackets[MAX_NUM_INPUT_AND_OUTPUT_OBJECTS];
        InputPacket *activeInputPackets[MAX_NUM_INPUT_AND_OUTPUT_OBJECTS];
        OutputPacket *activeOutputPackets[MAX_NUM_INPUT_AND_OUTPUT_OBJECTS];


        
        uint8_t numInputPackets = 0;
        uint8_t numOutputPackets = 0;

    public:

        UnityPicoComms(const char* _PicoID){
            PicoID = _PicoID;
        }

        void setSerialPort(SerialUART* port){
            SerialPort = port;
        }

        void begin(){
            pinMode(LED_PIN, OUTPUT);
            SerialPort->setFIFOSize(FIFO_SIZE);
            SerialPort->begin(baudRate);
            rp2040.wdt_begin(watchdogTimerLength);
            for(int i = 0; i < numOutputPackets; i++){
                activeOutputPackets[i]->update(*activeOutputPackets[i]);
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
                if(incomingPacketSize < 5){
                    // Serial.print("wrong packet size? ");
                    // Serial.println(incomingPacketSize);
                    // for(int i = 0; i < incomingPacketSize; i++){
                    //     Serial.print((int)IncomingPacket[i]);
                    //     Serial.print(" ");
                    // }
                    // Serial.println();
                    continue;
                }
                // printf("incoming packet of size %i\n", incomingPacketSize);
                // for(int i = 0; i < incomingPacketSize; i++){
                //     Serial.print((int)IncomingPacket[i]);
                //     Serial.print(" ");
                // }
                // Serial.println();
                
                if(!ValidateIncomingPacket()){
                    // Serial.println("invalid packet");
                    continue;
                }
                // printPacketHeader();

                uint8_t _designatorCode = IncomingPacket[0] & PICO_DESIGNATOR_MASK;
                if(picoDesignatorCode != _designatorCode){
                    picoDesignatorCode = _designatorCode;
                    uint8_t readableDesignatorCode = picoDesignatorCode >> 4;
                    // printf("Pico designator code: %i\n", readableDesignatorCode);
                }

                ProcessIncomingPacket(incomingPacketSize);
            }
        }

        void printPacketHeader(){
                printf("\n----\nmessage type: %i, designator code: %i, checksum: %i,\nfirst index: %i, last index: %i, data size: %i, data start offset: %i\n----\n",
                    PacketHeader.MessageType,
                    PacketHeader.PicoDesignatorCode,
                    PacketHeader.Checksum,
                    PacketHeader.FirstIndex,
                    PacketHeader.LastIndex,
                    PacketHeader.DataSize,
                    PacketHeader.DataStartOffset
                );
        }

        bool ValidateIncomingPacket(){
            int offsetIndex = 0;
            const int defaultHeaderSize = 4;
            PacketHeader.Checksum = IncomingPacket[1];
            PacketHeader.FirstIndex = IncomingPacket[2] & SMALL_INDEX_MASK;
            if (IncomingPacket[2] > SMALL_INDEX_MASK)
            {
                PacketHeader.FirstIndex += (IncomingPacket[3] << 7);
                offsetIndex++;
            }
            PacketHeader.LastIndex = IncomingPacket[3 + offsetIndex] & SMALL_INDEX_MASK;
            if (IncomingPacket[3 + offsetIndex] > SMALL_INDEX_MASK)
            {
                PacketHeader.LastIndex += (IncomingPacket[4 + offsetIndex] << 7);
                offsetIndex++;
            }

            if (PacketHeader.FirstIndex > PacketHeader.LastIndex) { return false; }
            PacketHeader.DataStartOffset = defaultHeaderSize + offsetIndex;

            PacketHeader.DataSize = PacketHeader.LastIndex - PacketHeader.FirstIndex + 1;
            uint8_t _checkSum = 0;
            uint8_t _dataSize = 0;
            for (int i = PacketHeader.DataStartOffset; i < PacketHeader.DataStartOffset + PacketHeader.DataSize; i++)
            {
                // printf("incoming packet %i: %i\n", i, IncomingPacket[i]);
                _checkSum += IncomingPacket[i];
                _dataSize++;
            }

            if (_checkSum != PacketHeader.Checksum) {
                // printf("got %i, calculated %i\n", PacketHeader.Checksum, _checkSum);
                return false;
            }
            else if(_dataSize != PacketHeader.DataSize){
                return false;
            }

            PacketHeader.MessageType = IncomingPacket[0] & MESSAGE_TYPE_MASK;
            PacketHeader.PicoDesignatorCode = IncomingPacket[0] & PICO_DESIGNATOR_MASK;
            return true;
        }


        void ProcessIncomingPacket(int incomingPacketSize)
        {
            
            if ((IncomingPacket[0] & PING_MASK) == PING_MASK)
            {
                respondToPing();
                return;
            }
            if (PacketHeader.MessageType == ID_REQUEST_MESSAGE)
            {
                sendPicoID();
                return;
            }
            
            for(int i = 0; i < numInputPackets; i++){
                if(activeInputPackets[i]->messageType == PacketHeader.MessageType){
                    if(activeInputPackets[i]->updateInputBuffer()){
                        SendPacket(activeInputPackets[i]->messageType, picoDesignatorCode, activeInputPackets[i]->inputBufferChecksum);
                    }
                    return;
                }
            }
            
            for(int i = 0; i < numOutputPackets; i++){
                if(activeOutputPackets[i]->messageType == PacketHeader.MessageType){
                    activeOutputPackets[i]->verifyChecksum();
                    return;
                }
            }
        }

        void handleOutputPackets(){
            for(int i = 0; i < numOutputPackets; i++){
                activeOutputPackets[i]->update(*activeOutputPackets[i]);
                // --- Comment out if we don't want to occasionally resend full pacekts ---
                if(activeOutputPackets[i]->isTimeToResendFullPacket()){
                    activeOutputPackets[i]->doUpdatesNeedToBeSent = true;
                    activeOutputPackets[i]->makeBufferIndicesCoverFullPacket();
                }
                // ------------------------------------------------------------------------

                if(activeOutputPackets[i]->doUpdatesNeedToBeSent){
                    if(millis() - activeOutputPackets[i]->sendPacketReattemptTimer < TIME_BETWEEN_REATTEMPTED_MESSAGE) continue;
                    
                    SendPacket(
                        activeOutputPackets[i]->messageType,
                        picoDesignatorCode,
                        activeOutputPackets[i]->IndexOfFirstChangedBufferElement,
                        activeOutputPackets[i]->IndexOfLastChangedBufferElement,
                        activeOutputPackets[i]->outputBuffer
                    );

                    activeOutputPackets[i]->sendPacketReattemptTimer = millis();
                }
            }
        }

        void respondToPing(){
            // Serial.println("Ping!");
            SendPacket(PING_MASK + picoDesignatorCode, 0); // 0 because no data, just resonding to ping
        }


        void addOutputPacket(const char* _name, uint8_t _messageType, size_t _size, void(*_update)(OutputPacket &outputPacket)){
            if(!isMessageTypeAvailable(_messageType, _name)){
                Error();
            }
            outputPackets[_messageType].initialize(_name, _messageType, _size, _update);
            activeOutputPackets[numOutputPackets++] = &outputPackets[_messageType];
        }

        void addInputPacket(const char* _name, uint8_t _messageType, size_t _size, void(*callback)(uint8_t*, size_t)){
            if(!isMessageTypeAvailable(_messageType, _name)){
                Error();
            }
            inputPackets[_messageType].initialize(_name, _messageType, _size, callback);
            activeInputPackets[numInputPackets++] = &inputPackets[_messageType];
        }

        bool isMessageTypeAvailable(byte _messageType, const char *_name){
            if(_messageType >= MAX_NUM_INPUT_AND_OUTPUT_OBJECTS) return false;

            for(int i = 0; i < numInputPackets; i++){
                if(activeInputPackets[i]->messageType == _messageType){
                    // printf("Error adding %s. Input packet %s already has messageType %i\n", _name, activeInputPackets[i]->name, _messageType);
                    return false;
                }
            }
            for(int i = 0; i < numOutputPackets; i++){
                if(activeOutputPackets[i]->messageType == _messageType){
                    // printf("Error adding %s. Output packet %s already has messageType %i\n", _name, activeOutputPackets[i]->name, _messageType);
                    return false;
                }
            }
            return true;
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
            OutgoingPacket[3 + dataOffset] = lastIndex & SMALL_INDEX_MASK;
            if (lastIndex > SMALL_INDEX_MASK)
            {
                OutgoingPacket[4 + dataOffset] = lastIndex >> 7;
                dataOffset++;
            }
            byte checksum = 0;
            for (int i = firstIndex; i <= lastIndex; i++)
            {
                OutgoingPacket[dataOffset + defaultHeaderSize + i - firstIndex] = buffer[i];

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