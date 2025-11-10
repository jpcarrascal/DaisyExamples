#include "daisysp.h"
#include "daisy_pod.h"

#define MAX_SIZE (48000 * 60 * 1) // 1 minute of floats at 48 khz (stereo-friendly)

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

void ResetBuffer();
void Controls();

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

    // start callback
    pod.StartAdc();
    pod.StartAudio(AudioCallback);

    while(1) {}
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
    //button1 pressed
    if(pod.button2.RisingEdge())
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
        rec  = !rec;
    }

    //button1 held
    if(pod.button2.TimeHeldMs() >= 1000 && res)
    {
        ResetBuffer();
        res = false;
    }

    //button2 pressed and not empty buffer
    if(pod.button1.RisingEdge() && !(!rec && first))
    {
        pos = loopStart; // Play from beginning
        play = !play;
        rec  = false;
    }

    if(pod.encoder.RisingEdge()) {
        encoderDown = false;  // Reset when encoder is released
    }

    if(pod.encoder.FallingEdge() && !encoderDown)
    {
        fwd = !fwd;
    }
    
    // Encoder long press resets speed
    if(pod.encoder.TimeHeldMs() >= 1000)
    {
        encoderDown = true;
        loopSpeed = 1.0;
        positionAccumulator = 0.0;
    }
    
    // Encoder turn controls speed
    int32_t encoder_inc = pod.encoder.Increment();
    if(encoder_inc > 0)
    {
        loopSpeed *= 1.1f;  // Increase speed by 10%
        if(loopSpeed > 4.0f) loopSpeed = 4.0f;  // Cap at 4x speed
    }
    else if(encoder_inc < 0)
    {
        loopSpeed *= 0.9f;  // Decrease speed by 10%
        if(loopSpeed < 0.25f) loopSpeed = 0.25f;  // Cap at 1/4 speed
    }
}

//Deals with analog controls
void Controls()
{
    pod.ProcessAnalogControls();
    pod.ProcessDigitalControls();

    float knob1Val = pod.knob1.Process();
    float knob2Val = pod.knob2.Process();
    
    // Only allow knob takeover after first loop is recorded
    if(!first) {
        // Knob1 takeover: becomes active when moved to 0-0.05 range
        if(knob1Val >= 0.0f && knob1Val <= 0.05f) {
            knob1Active = true;
        }
        
        // Knob2 takeover: becomes active when moved to 0.95-1.0 range
        if(knob2Val >= 0.95f && knob2Val <= 1.0f) {
            knob2Active = true;
        }
    }

    // Apply knob control only when active
    if(knob1Active) {
        loopStart = (int) (mod * knob1Val);
    }
    if(knob2Active) {
        loopLength = (int) ( (mod-loopStart) * knob2Val );
    }
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
