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
// float               inVol  = 0;
// float               loopVol= 0;
bool                res    = false;
bool                knob1Takeover = false;
bool                knob2Takeover = false;

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
    pos   = 0;
    len   = 0;
    knob1Takeover = false;
    knob2Takeover = false;
    
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

    if(pod.encoder.RisingEdge())
    {
        fwd = !fwd;
    }
}

//Deals with analog controls
void Controls()
{
    pod.ProcessAnalogControls();
    pod.ProcessDigitalControls();

    // Only allow knob control after first loop is recorded
    if(!first) {
        loopStart  = (int) (mod * pod.knob1.Process());
        loopLength = (int) ( (mod-loopStart) * pod.knob2.Process() );
    }
    if(loopLength < 1) loopLength = 1;

    // Clamp position if it's outside the new loop boundaries
    if(pos < loopStart || pos >= loopStart + loopLength) {
        pos = loopStart;
    }

    UpdateButtons();

    //leds
    pod.led1.Set(0, play == true, 0);
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
        if(fwd) {
            pos++;
            if(pos >= loopStart + loopLength) pos = loopStart;  // Wrap to loop start
        } else {
            pos--;
            if(pos < loopStart) pos = loopStart + loopLength - 1;  // Wrap to loop end
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
