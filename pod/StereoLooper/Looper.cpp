#include "daisysp.h"
#include "daisy_pod.h"

#define MAX_SIZE (48000 * 60 * 1) // 1 minute of floats at 48 khz (stereo-friendly)

// Phase 3: MIDI Control Change mappings
#define MIDI_CC_RECORD      64   // Record toggle
#define MIDI_CC_PLAY_TOGGLE 65   // Play toggle  
#define MIDI_CC_PLAY        66   // Play (always start)
#define MIDI_CC_STOP        67   // Stop (always stop)
#define MIDI_CC_DIRECTION   68   // Direction toggle
#define MIDI_CC_SPEED_RESET 69   // Speed reset to 1.0x
#define MIDI_CC_SPEED       70   // Speed control (0-127 → 0.25x-4x)
#define MIDI_CC_LOOP_START  71   // Loop start (0-127 → 0-1)
#define MIDI_CC_LOOP_LENGTH 72   // Loop length (0-127 → 0-1)
#define MIDI_CC_RESET       73   // Buffer reset

// Phase 3: MIDI Note mappings (alternative triggers)
#define MIDI_NOTE_RECORD    60   // C4 = Record toggle
#define MIDI_NOTE_PLAY      61   // C#4 = Play 
#define MIDI_NOTE_STOP      62   // D4 = Stop
#define MIDI_NOTE_DIRECTION 63   // D#4 = Direction toggle
#define MIDI_NOTE_RESET     64   // E4 = Buffer reset

// Channel constants for better readability
const int L = 0;  // Left channel index
const int R = 1;  // Right channel index

using namespace daisysp;
using namespace daisy;

static DaisyPod pod;

bool first = true;  //first loop (sets length)
bool rec   = false; //currently recording
bool play  = false; //currently playing
bool fwd   = true;  //forward direction

int                 pos = 0;
float DSY_SDRAM_BSS buf[MAX_SIZE][2];  // Stereo buffer [sample][channel] - L=0, R=1
int                 mod    = MAX_SIZE;
int                 len    = 0;
int                 loopStart  = 0;
int                 loopLength = MAX_SIZE;
float               loopSpeed = 1.0;    // Loop playback speed
float               positionAccumulator = 0.0;  // For fractional position tracking
uint32_t            loopBlinkTimer = 0;  // Timer for LED blink at loop start
// float               inVol  = 0;
// float               loopVol= 0;
bool                res    = false;
bool                encoderDown = false; // Encoder direction control
bool                knob1Active = false;  // Knob1 becomes active when in 0-0.05 range
bool                knob2Active = false;  // Knob2 becomes active when in 0.95-1 range
bool                midiLoopControl = false;  // Flag: MIDI has taken control of loop start/length

void ResetBuffer();
void Controls();

// MIDI handler - Phase 2
void updateMIDIControls();

// MIDI Function Declarations:
float midiToFloat(uint8_t midiValue);
float midiToSpeed(uint8_t midiValue);


// Control functions - Phase 1: Extract control logic
void finalizeFirstLoop();  // Helper function for first loop completion
void handleRecordToggle();
void handleRecordOn();
void handleRecordOff();
void handlePlayToggle();
void handlePlay();
void handleStop();
void handleDirectionToggle();
void handleReverse();
void handleForward();
void handleSpeedReset();
void handleSpeedChange(int32_t increment);
void handleKnob1(float value);
void handleKnob2(float value);
void handleReset();

void NextSamples(float&                               outputL,
                 float&                               outputR,
                 AudioHandle::InterleavingInputBuffer in,
                 size_t                               i);

static void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                          AudioHandle::InterleavingOutputBuffer out,
                          size_t                                size)
{
    float outputL = 0;  // Left channel output
    float outputR = 0;  // Right channel output

    Controls();

    for(size_t i = 0; i < size; i += 2)
    {
        // Process stereo samples
        NextSamples(outputL, outputR, in, i);

        // Output left and right channels separately
        out[i]     = outputL;   // Left channel output
        out[i + 1] = outputR;   // Right channel output
    }
}

int main(void)
{
    // initialize pod hardware and oscillator daisysp module

    pod.Init();
    pod.SetAudioBlockSize(4);
    ResetBuffer();

    // Initialize MIDI - Phase 2
    pod.midi.StartReceive();

    // start callback
    pod.StartAdc();
    pod.StartAudio(AudioCallback);

    // Main loop - Phase 2: Add MIDI processing
    while(1) {
        updateMIDIControls();  // Process MIDI events
    }
}

//Resets the buffer
void ResetBuffer()
{
    play  = false;
    rec   = false;
    first = true;
    fwd   = true;
    loopSpeed = 1.0;
    positionAccumulator = 0.0;
    loopBlinkTimer = 0;
    pos   = 0;
    len   = 0;
    loopStart = 0;                // Reset loop start to beginning
    loopLength = MAX_SIZE;        // Reset loop length to full buffer
    knob1Active = false;          // Reset knob takeover - knobs become inactive
    knob2Active = false;          // Reset knob takeover - knobs become inactive
    midiLoopControl = false;      // Reset control back to physical knobs
    
    // Clear both left and right channel buffers
    for(int i = 0; i < mod; i++)
    {
        buf[i][L] = 0;  // Clear left channel
        buf[i][R] = 0;  // Clear right channel
    }
    mod = MAX_SIZE;
}

void UpdateButtons()
{
    //button2 pressed - Record toggle
    if(pod.button2.RisingEdge())
    {
        handleRecordToggle();
    }

    //button2 held - Reset
    if(pod.button2.TimeHeldMs() >= 1000 && res)
    {
        handleReset();
        res = false;
    }

    //button1 pressed - Play toggle (and not empty buffer)
    if(pod.button1.RisingEdge() && !(!rec && first))
    {
        handlePlayToggle();
    }

    // Encoder button handling
    if(pod.encoder.RisingEdge()) {
        encoderDown = false;  // Reset when encoder is released
    }

    if(pod.encoder.FallingEdge() && !encoderDown)
    {
        handleDirectionToggle();
    }
    
    // Encoder long press resets speed
    if(pod.encoder.TimeHeldMs() >= 1000)
    {
        encoderDown = true;
        handleSpeedReset();
    }
    
    // Encoder turn controls speed
    int32_t encoder_inc = pod.encoder.Increment();
    if(encoder_inc != 0)
    {
        handleSpeedChange(encoder_inc);
    }
}

//Deals with analog controls
void Controls()
{
    pod.ProcessAnalogControls();
    pod.ProcessDigitalControls();

    float knob1Val = pod.knob1.Process();
    float knob2Val = pod.knob2.Process();
    
    // Handle knobs through control functions
    // Physical knobs only active when MIDI doesn't have loop control
    handleKnob1(knob1Val);
    handleKnob2(knob2Val);
    
    if(loopLength < 1) loopLength = 1;

    // Clamp position if it's outside the new loop boundaries
    if(pos < loopStart || pos >= loopStart + loopLength) {
        pos = loopStart;
    }

    UpdateButtons();

    //leds
    // LED1 blinks off for 100ms at loop start to show speed
    bool led1_on = play;
    if(loopBlinkTimer > 0 && (System::GetNow() - loopBlinkTimer) < 100) {
        led1_on = false;  // Turn off for 100ms after loop restart
    }
    pod.led1.Set(0, led1_on, 0);
    pod.led2.Set(rec == true, 0, 0);

    pod.UpdateLeds();
}

// MIDI handler - Phase 4: Implement MIDI control
void updateMIDIControls()
{
    while(pod.midi.HasEvents()) {
        MidiEvent event = pod.midi.PopEvent();
        
        // Handle Control Change messages
        if(event.type == ControlChange) {
            uint8_t ccNumber = event.data[0];  // CC number
            uint8_t ccValue = event.data[1];   // CC value
            
            switch(ccNumber) {
                case MIDI_CC_RECORD:
                    if(ccValue > 63) {
                        handleRecordOn();   // Values 64-127 = Record ON
                    } else {
                        handleRecordOff();  // Values 0-63 = Record OFF
                    }
                    break;
                    
                case MIDI_CC_PLAY_TOGGLE:
                    if(ccValue >= 64) handlePlayToggle();
                    break;
                    
                case MIDI_CC_PLAY:
                    if(ccValue >= 64) handlePlay();
                    break;
                    
                case MIDI_CC_STOP:
                    if(ccValue >= 64) handleStop();
                    break;
                    
                case MIDI_CC_DIRECTION:
                    if(ccValue > 63) {
                        handleReverse();  // Values 64-127 = Reverse
                    } else {
                        handleForward();  // Values 0-63 = Forward
                    }
                    break;
                    
                case MIDI_CC_SPEED_RESET:
                    if(ccValue >= 64) handleSpeedReset();
                    break;
                    
                case MIDI_CC_SPEED:
                    loopSpeed = midiToSpeed(ccValue);
                    positionAccumulator = 0.0;  // Reset accumulator when speed changes
                    break;
                    
                case MIDI_CC_LOOP_START:
                    // Direct MIDI control - no knob takeover logic needed
                    loopStart = (int)(mod * midiToFloat(ccValue));
                    // Clamp to valid range
                    if(loopStart >= mod - 1) loopStart = mod - 1;
                    // MIDI takes control of loop parameters
                    midiLoopControl = true;
                    break;
                    
                case MIDI_CC_LOOP_LENGTH:
                    // Direct MIDI control - set length relative to remaining buffer
                    loopLength = (int)((mod - loopStart) * midiToFloat(ccValue));
                    if(loopLength < 1) loopLength = 1;
                    // MIDI takes control of loop parameters
                    midiLoopControl = true;
                    break;
                    
                case MIDI_CC_RESET:
                    if(ccValue >= 64) handleReset();
                    break;
            }
        }
        
        // Handle Note On messages
        if(event.type == NoteOn && event.data[1] > 0) {  // Velocity > 0
            uint8_t noteNumber = event.data[0];  // Note number
            
            switch(noteNumber) {
                case MIDI_NOTE_RECORD:
                    handleRecordToggle();
                    break;
                    
                case MIDI_NOTE_PLAY:
                    handlePlay();
                    break;
                    
                case MIDI_NOTE_STOP:
                    handleStop();
                    break;
                    
                case MIDI_NOTE_DIRECTION:
                    handleDirectionToggle();
                    break;
                    
                case MIDI_NOTE_RESET:
                    handleReset();
                    break;
            }
        }
    }
}

// Phase 3: MIDI value conversion helpers
float midiToFloat(uint8_t midiValue) 
{
    return (float)midiValue / 127.0f;  // 0-127 → 0.0-1.0
}

float midiToSpeed(uint8_t midiValue) 
{
    // 0-127 → 0.25-4.0, with 64 = 1.0 (normal speed)
    if(midiValue == 64) return 1.0f;
    if(midiValue < 64) {
        return 0.25f + (midiValue / 64.0f) * 0.75f;  // 0-64 → 0.25-1.0
    } else {
        return 1.0f + ((midiValue - 64) / 63.0f) * 3.0f;  // 65-127 → 1.0-4.0
    }
}

// Helper function for first loop completion - eliminates duplication
void finalizeFirstLoop()
{
    if(first && rec)
    {
        first = false;
        mod   = len;
        loopLength = len;
        len   = 0;
    }
    res  = true;
    play = true;
}

// Control functions - Phase 1: Extract control logic
void handleRecordToggle()
{
    finalizeFirstLoop();
    rec = !rec;
}

void handleRecordOn()
{
    finalizeFirstLoop();
    rec = true;
}

void handleRecordOff()
{
    finalizeFirstLoop();
    rec = false;
}


void handlePlayToggle()
{
    pos = loopStart; // Play from beginning
    play = !play;
    rec  = false;
}

void handlePlay()
{
    pos = loopStart; // Play from beginning
    play = true;
    rec  = false;
}

void handleStop()
{
    pos = loopStart; // Play from beginning
    play = false;
    rec  = false;
}

void handleDirectionToggle()
{
    fwd = !fwd;
}

void handleReverse()
{
    fwd = false;
}

void handleForward()
{
    fwd = true;
}

void handleSpeedReset()
{
    loopSpeed = 1.0;
    positionAccumulator = 0.0;
}

void handleSpeedChange(int32_t increment)
{
    if(increment > 0)
    {
        loopSpeed *= 1.1f;  // Increase speed by 10%
        if(loopSpeed > 4.0f) loopSpeed = 4.0f;  // Cap at 4x speed
    }
    else if(increment < 0)
    {
        loopSpeed *= 0.9f;  // Decrease speed by 10%
        if(loopSpeed < 0.25f) loopSpeed = 0.25f;  // Cap at 1/4 speed
    }
}

void handleKnob1(float value)
{
    // Only work when MIDI doesn't have control of loop parameters
    if(!midiLoopControl) {
        // Only allow knob takeover after first loop is recorded
        if(!first) {
            // Knob1 takeover: becomes active when moved to 0-0.05 range
            if(value >= 0.0f && value <= 0.05f) {
                knob1Active = true;
            }
        }

        // Apply knob control only when active
        if(knob1Active) {
            loopStart = (int) (mod * value);
        }
    }
}

void handleKnob2(float value)
{
    // Only work when MIDI doesn't have control of loop parameters
    if(!midiLoopControl) {
        // Only allow knob takeover after first loop is recorded
        if(!first) {
            // Knob2 takeover: becomes active when moved to 0.95-1.0 range
            if(value >= 0.95f && value <= 1.0f) {
                knob2Active = true;
            }
        }

        // Apply knob control only when active
        if(knob2Active) {
            loopLength = (int) ( (mod-loopStart) * value );
        }
    }
}

void handleReset()
{
    ResetBuffer();
}

void WriteBuffer(AudioHandle::InterleavingInputBuffer in, size_t i)
{
    // Write to both left and right channel buffers
    // in[i] = left channel, in[i+1] = right channel
    buf[pos][L] = buf[pos][L] * 0.9 + in[i] * 0.9;      // Left channel overdub
    buf[pos][R] = buf[pos][R] * 0.9 + in[i + 1] * 0.9;  // Right channel overdub
    
    if(first)
    {
        len++;
    }
}

void NextSamples(float&                               outputL,
                 float&                               outputR,
                 AudioHandle::InterleavingInputBuffer in,
                 size_t                               i)
{
    // Record to both channels if recording is active
    if(rec)
    {
        WriteBuffer(in, i);
    }

    // Read from both channel buffers using named constants
    outputL = buf[pos][L];  // Left channel output
    outputR = buf[pos][R];  // Right channel output

    //automatic looptime
    if(len >= MAX_SIZE)
    {
        first = false;
        mod   = MAX_SIZE;
        len   = 0;
    }

    // Advance playback position if playing
    if(play)
    {
        positionAccumulator += loopSpeed;
        
        if(fwd) {
            while(positionAccumulator >= 1.0f) {
                pos++;
                positionAccumulator -= 1.0f;
                if(pos >= loopStart + loopLength) {
                    pos = loopStart;
                    loopBlinkTimer = System::GetNow();  // Start blink timer when loop restarts
                }
            }
        } else {
            while(positionAccumulator >= 1.0f) {
                pos--;
                positionAccumulator -= 1.0f;
                if(pos < loopStart) {
                    pos = loopStart + loopLength - 1;
                    loopBlinkTimer = System::GetNow();  // Start blink timer when loop restarts
                }
            }
        }
    }

    // Apply dry/wet mix to both channels
    if(!rec)
    {
        // outputL = outputL * loopVol + in[i] * inVol;       // Left channel mix
        // outputR = outputR * loopVol + in[i + 1] * inVol;  // Right channel mix
        outputL = outputL + in[i];       // Left channel mix
        outputR = outputR + in[i + 1];  // Right channel mix
    }
}
