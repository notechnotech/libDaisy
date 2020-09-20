#pragma once
#ifndef DSY_MIDI_H
#define DSY_MIDI_H

#include <stdint.h>
#include <stdlib.h>
#include "per/uart.h"
#include "util/ringbuffer.h"

#ifndef MAX_MIDI_MESSAGE_BUFFER_SIZE
#define MAX_MIDI_MESSAGE_BUFFER_SIZE 256
#endif

namespace daisy
{
/** @addtogroup external 
    @{ 
*/


/** Parsed from the Status Byte, these are the common Midi Messages that can be handled. \n
At this time only 3-byte messages are correctly parsed into MidiEvents.
*/
enum MidiMessageType
{
    NoteOff                = 0x80,
    NoteOn                 = 0x90,
    PolyphonicKeyPressure  = 0xA0,
    ControlChange          = 0xB0,
    ProgramChange          = 0xC0,
    ChannelPressure        = 0xD0,
    PitchBend              = 0xE0,
    SysexStart             = 0xF0,
    SongPositionPointer    = 0xF1,
    SongSelect             = 0xF2,
    TuneRequest            = 0xF6,
    SysexEnd               = 0xF7,
    RealTimeTimingClock    = 0xF8,
    RealTimeStart          = 0xFA,
    RealTimeContinue       = 0xFB,
    RealTimeStop           = 0xFC,
    RealTimmeActiveSensing = 0xFE,
    RealTimeSystemReset    = 0xFF,
    //	MessageLast
};

/** Struct containing note, and velocity data for a given channel.
Can be made from MidiEvent
*/
struct NoteOnEvent
{
    int     channel;  /**< & */
    uint8_t note;     /**< & */
    uint8_t velocity; /**< & */
};

/** Struct containing note, and velocity data for a given channel.
Can be made from MidiEvent
*/
struct NoteOffEvent
{
    int     channel;  /**< & */
    uint8_t note;     /**< & */
    uint8_t velocity; /**< & */
};

/** Struct containing control number, and value for a given channel.
Can be made from MidiEvent
*/
struct ControlChangeEvent
{
    int     channel;        /**< & */
    uint8_t control_number; /**< & */
    uint8_t value;          /**< & */
};


struct ProgramChangeEvent
{
    int     channel;        /**< & */
    uint8_t program_number; /**< & */
};


/** Struct containing note, and velocity data for a given channel.
Can be made from MidiEvent
*/
struct PolyphonicKeyPressureEvent
{
    int     channel;  /**< & */
    uint8_t note;     /**< & */
    uint8_t pressure; /**< & */
};


struct ChannelPressureEvent
{
    int     channel;  /**< & */
    uint8_t pressure; /**< & */
};


struct PitchBendEvent
{
    int      channel;    /**< & */
    uint16_t pitch_bend; /**< & */
};


struct RealTimeEvent
{
    enum class EventType
    {
        RealTimeTimingClock,
        RealTimeStart,
        RealTimeContinue,
        RealTimeStop,
        RealTimmeActiveSensing,
        RealTimeSystemReset
    };

    EventType real_time_event;
};

/** Simple MidiEvent with message type, channel, and data[2] members.
*/
struct MidiEvent
{
    MidiMessageType type;                              
    int             channel;                            /* -1 if channel is not relavent to the MidiEvent (ie, sysex messages) */
	uint8_t         data[MAX_MIDI_MESSAGE_BUFFER_SIZE];
    size_t          data_size;

    /** Returns the data within the MidiEvent as a NoteOnEvent struct */
    NoteOnEvent AsNoteOn()
    {
        NoteOnEvent m;
        m.channel  = channel;
        m.note     = data[1];
        m.velocity = data[2];
        return m;
    }

	/** Alternative Way to get NoteOnEvent */
	bool AsNoteOn(NoteOnEvent & m)
	{
        if(type == MidiMessageType::NoteOn)
		{
            m.channel  = channel;
            m.note     = data[1];
            m.velocity = data[2];
            return true;
		}
        return false;
	}

    /** Returns the data within the MidiEvent as a ControlChangeEvent struct.*/
    ControlChangeEvent AsControlChange()
    {
        ControlChangeEvent m;
        m.channel        = channel;
        m.control_number = data[1];
        m.value          = data[2];
        return m;
    }

    /** A way to get a ControlChangeEvent struct populated. returns false if message is note a ControlChange */
    bool AsControlChange(ControlChangeEvent & m)
    {
        if(type == MidiMessageType::ControlChange)
		{
            m.channel        = channel;
            m.control_number = data[1];
            m.value          = data[2];
            return true;
		}
        return false;
    }

	ProgramChangeEvent AsProgramChangeEvent()
    {
        ProgramChangeEvent m;
        m.channel        = channel;
        m.program_number = data[1];
        return m;
    };

	bool AsProgramChangeEvent(ProgramChangeEvent &m)
    {
        if(type == MidiMessageType::ProgramChange)
        {
            m.channel        = channel;
            m.program_number = data[1];
            return true;
        }
        return false;
    };

	PolyphonicKeyPressureEvent AsPolyphonicKeyPressureEvent()
    {
        PolyphonicKeyPressureEvent m;
        m.channel  = channel;
        m.note     = data[1];
        m.pressure = data[2];
        return m;
    };

	bool AsPolyphonicKeyPressureEvent(PolyphonicKeyPressureEvent &m)
    {
        if(type == MidiMessageType::PolyphonicKeyPressure)
        {
            m.channel		= channel;
			m.note			= data[1];
            m.pressure		= data[2];
            return true;
        }
        return false;
    };

	ChannelPressureEvent AsChannelPressureEvent()
    {
        ChannelPressureEvent m;
        m.channel  = channel;
        m.pressure = data[1];
        return m;
    };


    bool AsChannelPressureEvent(ChannelPressureEvent &m)
    {
        if(type == MidiMessageType::ChannelPressure)
        {
            m.channel  = channel;
            m.pressure = data[1];
            return true;
        }
        return false;
    };

	PitchBendEvent AsPitchBendEvent()
    {
        PitchBendEvent m;
        m.channel  = channel;
		// TODO: FIX ME, this needs to put data[1] and data[2] into  uint16_t....
        m.pitch_bend = data[1]  ;
        return m;
    };

    bool AsPitchBendEvent(PitchBendEvent &m)
    {
        if(type == MidiMessageType::PitchBend)
        {
            m.channel  = channel;
            // TODO: FIX ME, this needs to put data[1] and data[2] into  uint16_t....
            m.pitch_bend = data[1];
            return true;
        }
        return false;
    };

};

/** 
    @brief Simple MIDI Handler \n 
    Parses bytes from an input into valid MidiEvents. \n 
    The MidiEvents fill a FIFO queue that the user can pop messages from.
    @author shensley
    @date March 2020
*/
class MidiHandler
{
  public:
    MidiHandler() {}
    ~MidiHandler() {}
    /** Input and Output can be configured separately
    Multiple Input modes can be selected by OR'ing the values.
    */
    enum MidiInputMode
    {
        INPUT_MODE_NONE    = 0x00, /**< & */
        INPUT_MODE_UART1   = 0x01, /**< & */
        INPUT_MODE_USB_INT = 0x02, /**< & */
        INPUT_MODE_USB_EXT = 0x04, /**< & */
    };
    /** Output mode */
    enum MidiOutputMode
    {
        OUTPUT_MODE_NONE    = 0x00, /**< & */
        OUTPUT_MODE_UART1   = 0x01, /**< & */
        OUTPUT_MODE_USB_INT = 0x02, /**< & */
        OUTPUT_MODE_USB_EXT = 0x04, /**< & */
    };


    /** Initializes the MidiHandler 
    \param in_mode Input mode
    \param out_mode Output mode
     */
    void Init(MidiInputMode in_mode, MidiOutputMode out_mode);

    /** Starts listening on the selected input mode(s). MidiEvent Queue will begin to fill, and can be checked with */
    void StartReceive();

    /** Start listening */
    void Listen();

    /** Feed in bytes to state machine from a queue.
    Populates internal FIFO queue with MIDI Messages
    For example with uart:
    midi.Parse(uart.PopRx());
    \param byte &
    */
    void Parse(uint8_t byte);

    /** Checks if there are unhandled messages in the queue 
    \return True if there are events to be handled, else false.
     */
    bool HasEvents() const { return event_q_.readable(); }


    /** Pops the oldest unhandled MidiEvent from the internal queue
    \return The event to be handled
     */
    MidiEvent PopEvent() { return event_q_.Read(); }

    /** SendMessage
    Send raw bytes as message
    */
    void SendMessage(uint8_t *bytes, size_t size);

	void SetInterpretZeroVelocityNoteOnAsNoteOffMessage(bool setInterpretZeroVelocityNoteOnAsNoteOffMessage);

  private:

	  static int16_t GetExpectedMessageSizeForMidiMessageType(MidiMessageType midiMessageType)	// -1 if unknown
	  {
          switch(midiMessageType)
          {
              case NoteOff:
              case NoteOn:
              case PolyphonicKeyPressure:
              case ControlChange:
              case ProgramChange:
              case ChannelPressure:
              case PitchBend:
              case SysexStart:
              case SongPositionPointer:
              case SongSelect: 
				  return 2;
              case TuneRequest:
              case SysexEnd: 
				  return 0;
              case RealTimeTimingClock:
              case RealTimeStart:
              case RealTimeContinue:
              case RealTimeStop:
              case RealTimmeActiveSensing:
              case RealTimeSystemReset:
				  return 1;
          }
          return -1;
	  }
	  class ParserState
	  {

        public:
          enum State
          {
              ParserEmpty,
           //   ParserHasStatus,
              ParserIsExpectingDataBytes,
          };

		  inline void ResetState()
		  { 
			  state_ = ParserEmpty;
              numBytesRecieved_ = 0;
		  }

		  inline void SetReceivedStatusByte(MidiMessageType messageType, bool isRunningMessage = false) // set -1 if unknown (sysex)
		  {
              numBytesRecieved_ = isRunningMessage  ? messageType == MidiMessageType::SysexStart ? 0 : 1 : 1;
			  messageType_ = messageType;
          }
		  inline void IncrementDataBytesReceieved()
          { 
			  state_ = ParserIsExpectingDataBytes;
			  numBytesRecieved_++;
          }
		  inline bool HasRecievedExpectedMessageData()
		  {
              return (MidiHandler::GetExpectedMessageSizeForMidiMessageType(messageType_) >= numBytesRecieved_);
          }
		  inline State GetState()
		  { 
			  return state_;
		  }
          inline uint16_t GetNumBytesRecieved() 
		  { 
			  return numBytesRecieved_;
		  }

	  private:
          State   state_;
          uint8_t numBytesRecieved_ = 0;
          MidiMessageType messageType_;
    };
    MidiInputMode              in_mode_;
    MidiOutputMode             out_mode_;
    UartHandler                uart_;
    ParserState                pstate_;
    MidiEvent                  incoming_message_;
    RingBuffer<MidiEvent, 256> event_q_;
    uint32_t                   last_read_; // time of last byte
    MidiMessageType            running_status_;
    bool                       interpret_zero_velocity_note_on_as_note_off_message_;
};

/** @} */
} // namespace daisy
#endif
