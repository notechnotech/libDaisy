#include "hid/midi.h"
#include "sys/system.h"
#include <memory>
using namespace daisy;

// Masks to check for message type, and byte content
const uint8_t kStatusByteMask   = 0x80;
const uint8_t kMessageMask      = 0x70;
const uint8_t kDataByteMask     = 0x7F;
const uint8_t kSystemCommonMask = 0xF0;
const uint8_t kChannelMask      = 0x0F;
const uint8_t kRealTimeMask     = 0xF8;


// TODO:
// - provide an input interface so USB or UART data can be passed in.
//     this could even bue as simple as a buffer/new flag.
void MidiHandler::Init(MidiInputMode in_mode, MidiOutputMode out_mode)
{
    in_mode_  = in_mode;
    out_mode_ = out_mode;
    uart_.Init();
    event_q_.Init();
    //incoming_message_.type = MessageLast;
    pstate_.ResetState();
}

void MidiHandler::StartReceive()
{
    if(in_mode_ & INPUT_MODE_UART1)
    {
        uart_.StartRx();
    }
}

void MidiHandler::Listen()
{
    uint32_t now;
    now = dsy_system_getnow();
    if(in_mode_ & INPUT_MODE_UART1)
    {
        while(uart_.Readable())
        {
            last_read_ = now;
            Parse(uart_.PopRx());
        }

        // In case of UART Error, (particularly
        //  overrun error), UART disables itself.
        // Flush the buff, and restart.
        if(!uart_.RxActive())
        {
            pstate_.ResetState();
            uart_.FlushRx();
            StartReceive();
        }
    }
}


void MidiHandler::Parse(uint8_t byte)
{
    switch(pstate_.GetState())
    {
        case ParserState::State::ParserEmpty:
        {
            // check byte for valid Status Byte
            if(byte & kStatusByteMask)
            {
				incoming_message_.channel = byte & kChannelMask;
                incoming_message_.type = static_cast<MidiMessageType>((byte & 0xF0));
				pstate_.SetReceivedStatusByte(incoming_message_.type);
                
				// Mark this status byte as running_status
                running_status_ = incoming_message_.type;
                if(pstate_.HasRecievedExpectedMessageData())
                {
                    event_q_.Write(incoming_message_);
                    pstate_.ResetState();
                }
            }
            else
            {
                // Handle as running status
                incoming_message_.type = running_status_;
                if(incoming_message_.channel >= 0)
				{
                    incoming_message_.data[0] = running_status_ & incoming_message_.channel; 
				}
				else
				{
                    incoming_message_.data[0] = running_status_;
				}
                pstate_.SetReceivedStatusByte(incoming_message_.type);
                incoming_message_.data[pstate_.GetNumBytesRecieved()] = byte;
                pstate_.IncrementDataBytesReceieved();
                if(pstate_.HasRecievedExpectedMessageData())
                {
                    event_q_.Write(incoming_message_);
                }
            }
        }
        break;
        case ParserState::State::ParserIsExpectingDataBytes:
            incoming_message_.data[pstate_.GetNumBytesRecieved()] = byte;
            pstate_.IncrementDataBytesReceieved();
			if (incoming_message_.type == MidiMessageType::SysexStart)
			{
                if(pstate_.GetNumBytesRecieved() == MAX_MIDI_MESSAGE_BUFFER_SIZE)
                {
                    event_q_.Write(incoming_message_);
                    pstate_.ResetState();
                    //pstate_.SetRecivedStatusByte(1);
                }
			}
            else if(pstate_.HasRecievedExpectedMessageData())
            {
                event_q_.Write(incoming_message_);
                pstate_.ResetState();
            }
            break;
        default:
			break;
    }


}

void MidiHandler::SendMessage(uint8_t *bytes, size_t size)
{
    if(out_mode_ == OUTPUT_MODE_UART1)
    {
        uart_.PollTx(bytes, size);
    }
}
