/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2011 Steven Lovegrove
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef PANGOLIN_FIREWIRE_H
#define PANGOLIN_FIREWIRE_H

#include <pangolin/pangolin.h>
#include <pangolin/video.h>

#include <dc1394/dc1394.h>

#ifndef _WIN32
#include <unistd.h>
#endif



namespace pangolin
{

PANGOLIN_EXPORT
std::string Dc1394ColorCodingToString(dc1394color_coding_t coding);

PANGOLIN_EXPORT
dc1394color_coding_t Dc1394ColorCodingFromString(std::string coding);

PANGOLIN_EXPORT
void Dc1394ModeDetails(dc1394video_mode_t mode, unsigned& w, unsigned& h, std::string& format );

class PANGOLIN_EXPORT FirewireFrame
{
    friend class FirewireVideo;
public:
    bool isValid() { return frame; }
    unsigned char* Image() { return frame ? frame->image : 0; }
    unsigned Width() const { return frame ? frame->size[0] : 0; }
    unsigned Height() const { return frame ? frame->size[1] : 0; }
    unsigned ImageBytes() const { return frame ? frame->image_bytes : 0; }
    int FramesBehind() const { return frame ? frame->frames_behind : -1; }
    unsigned Timestamp() const { return frame ? frame->timestamp : 0; }
    int Id() const { return frame ? frame->id : -1; }
protected:
    FirewireFrame(dc1394video_frame_t* frame) : frame(frame) {}
    dc1394video_frame_t *frame;
};

struct PANGOLIN_EXPORT Guid
{
    Guid(uint64_t guid):guid(guid){}
    uint64_t guid;
};

struct MetaData {
    uint32_t flags;
    unsigned int brightness;
    unsigned int auto_exposure;
    unsigned int whitebalance_u_b, whitebalance_v_r;
    uint32_t timestamp, frame_count;
    uint32_t shutterQuant, gain;
    float shutterAbs;

    //TODO: Add strobe, GPIO and ROI functionality if needed.
    uint32_t strobe, gpio, roi;

    void copy_from( MetaData *in ) {
        flags=in->flags;
        brightness=in->brightness;
        auto_exposure=in->auto_exposure;
        whitebalance_u_b=in->whitebalance_u_b;
        whitebalance_v_r=in->whitebalance_v_r;
        timestamp=in->timestamp;
        frame_count=in->frame_count;
        shutterQuant=in->shutterQuant;
        gain=in->gain;
        shutterAbs=in->shutterAbs;
        strobe = in->strobe;
        gpio=in->gpio;
        roi=in->roi;
    }
};

typedef enum {
    META_TIMESTAMP = 1,
    META_GAIN = 2,
    META_SHUTTER = 4,
    META_BRIGHTNESS = 8,
    META_EXPOSURE = 16,
    META_WHITE_BALANCE = 32,
    META_FRAME_COUNTER = 64,
    META_STROBE = 128,
    META_GPIO_PIN_STATE = 256,
    META_ROI_POSITION = 512,
    META_ALL = 1023
} meta_flags;

class PANGOLIN_EXPORT FirewireVideo : public VideoInterface
{
public:
    const static int MAX_FR = -1;
    const static int EXT_TRIG = -1;
    const static uint32_t GPIO_CTRL_PIN0 = 0x1110;
    const static uint32_t GPIO_CTRL_PIN1 = 0x1120;
    const static uint32_t GPIO_CTRL_PIN2 = 0x1130;
    const static uint32_t GPIO_CTRL_PIN3 = 0x1140;
    
    FirewireVideo(
            unsigned deviceid = 0,
            dc1394video_mode_t video_mode = DC1394_VIDEO_MODE_640x480_RGB8,
            dc1394framerate_t framerate = DC1394_FRAMERATE_30,
            dc1394speed_t iso_speed = DC1394_ISO_SPEED_400,
            int dma_buffers = 10
            );
    
    FirewireVideo(
            Guid guid,
            dc1394video_mode_t video_mode = DC1394_VIDEO_MODE_640x480_RGB8,
            dc1394framerate_t framerate = DC1394_FRAMERATE_30,
            dc1394speed_t iso_speed = DC1394_ISO_SPEED_400,
            int dma_buffers = 10
            );
    
    FirewireVideo(
            Guid guid,
            dc1394video_mode_t video_mode,
            float framerate,
            uint32_t width, uint32_t height,
            uint32_t left, uint32_t top,
            dc1394speed_t iso_speed,
            int dma_buffers, bool reset_at_boot=false
            );
    
    FirewireVideo(
            unsigned deviceid,
            dc1394video_mode_t video_mode,
            float framerate,
            uint32_t width, uint32_t height,
            uint32_t left, uint32_t top,
            dc1394speed_t iso_speed,
            int dma_buffers, bool reset_at_boot=false
            );
    
    ~FirewireVideo();
    
    //! Implement VideoInput::Start()
    void Start();
    
    //! Implement VideoInput::Stop()
    void Stop();

//! Stop the video stream before using One-shot mode
  void StopForOneShot();

  //! Clear the DMA buffer
  void FlushDMABuffer();

  //! Grab one shot (iso-transmission must be off - call StopForOneShot first)
  bool GrabOneShot(unsigned char* image);

  //! Check to see if camera is one-shot capable
  bool CheckOneShotCapable();

  //! set the meta data flags to be included in image data
  void SetMetaDataFlags(  int flags );

  //! return the current meta data flags from camera
  uint32_t GetMetaDataFlags();

  //! read the meta data from an image according to meta flags
  void ReadMetaData( unsigned char *image, MetaData *metaData );
  float ReadShutter( unsigned char *image );

  //! create lookup table to convert quantised shutter values to absolute values
  void CreateShutterLookupTable();

    //! Implement VideoInput::SizeBytes()
    size_t SizeBytes() const;

    //! Implement VideoInput::Streams()
    const std::vector<StreamInfo>& Streams() const;
    
    //! Implement VideoInput::GrabNext()
    bool GrabNext( unsigned char* image, bool wait = true );
    
    //! Implement VideoInput::GrabNewest()
    bool GrabNewest( unsigned char* image, bool wait = true );

    //! (deprecated: use Streams[i].Width())
    //! Return image width
    unsigned Width() const { return width; }
    
    //! (deprecated: use Streams[i].Height())
    //! Return image height
    unsigned Height() const { return height; }
    
    //! Return object containing reference to image data within
    //! DMA buffer. The FirewireFrame must be returned to
    //! signal that it can be reused with a corresponding PutFrame()
    FirewireFrame GetNext(bool wait = true);
    
    //! Return object containing reference to newest image data within
    //! DMA buffer discarding old images. The FirewireFrame must be
    //! returned to signal that it can be reused with a corresponding PutFrame()
    FirewireFrame GetNewest(bool wait = true);
    
    //! Return FirewireFrame object. Data held by FirewireFrame is
    //! invalidated on return.
    void PutFrame(FirewireFrame& frame);
    
    //! return absolute shutter value
    float GetShutterTime() const;
    
    //! set absolute shutter value
    void SetShutterTime(float val);
    
    //! set auto shutter value
    void SetAutoShutterTime();

  void SetShutterTimeManual();

  //! return the quantised gain
  int GetGainQuant() const;
    
    //! return absolute gain value
    float GetGain() const;
    
    //! set absolute gain value
    void SetGain(float val);
    
    //! set auto gain value
    void SetAutoGain();
    
    //! return absolute brightness value
    float GetBrightness() const;
    
    //! set absolute brightness value
    void SetBrightness(float val);
    
    //! set auto brightness
    void SetAutoBrightness();
    
    //! return absolute gamma value
    float GetGamma() const;
    
    //! return quantised shutter value
    void SetShutterTimeQuant(int shutter);

  //! set the white balance
  void SetWhiteBalance(unsigned int u_b_value, unsigned int v_r_value);

  //! get the white balance
  void GetWhiteBalance(unsigned int *Blue_U_val, unsigned int *Red_V_val);

  //! set the white balance to single shot auto
  void SetSingleAutoWhiteBalance();

  //! return the quantised shutter time
  int GetShutterTimeQuant() const;
    
    //! set the trigger to internal, i.e. determined by video mode
    void SetInternalTrigger();
    
    //! set the trigger to external
    void SetExternalTrigger(
            dc1394trigger_mode_t mode=DC1394_TRIGGER_MODE_0,
            dc1394trigger_polarity_t polarity=DC1394_TRIGGER_ACTIVE_HIGH,
            dc1394trigger_source_t source=DC1394_TRIGGER_SOURCE_0
            );
    
    //! set a camera register
    void SetRegister(uint64_t offset, uint32_t value);
    
    //! read camera register
    uint32_t GetRegister(uint64_t offset);
    
    //! set a camera control register
    void SetControlRegister(uint64_t offset, uint32_t value);
    
    //! read camera control register
    uint32_t GetControlRegister(uint64_t offset);
    
protected:
    void init_stream_info();
    
    void init_camera(
            uint64_t guid, int dma_frames,
            dc1394speed_t iso_speed,
            dc1394video_mode_t video_mode,
            dc1394framerate_t framerate
            );
    
    void init_format7_camera(
            uint64_t guid, int dma_frames,
            dc1394speed_t iso_speed,
            dc1394video_mode_t video_mode,
            float framerate,
            uint32_t width, uint32_t height,
            uint32_t left, uint32_t top, bool reset_at_boot
            );
    
    static int nearest_value(int value, int step, int min, int max);
    static double bus_period_from_iso_speed(dc1394speed_t iso_speed);
    
    size_t frame_size_bytes;
    std::vector<StreamInfo> streams;
    
    bool running;
    dc1394camera_t *camera;
    unsigned width, height, top, left;
    //dc1394featureset_t features;
    dc1394_t * d;
    dc1394camera_list_t * list;
    mutable dc1394error_t err;

  uint32_t meta_data_flags;
  float* shutter_lookup_table;
    
};

}

#endif // PANGOLIN_FIREWIRE_H
