// Copyright 2017 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_heap_caps.h"
#include "anim.h"
#include "val2pwm.h"
#include "i2s_parallel.h"

#include "driver/gpio.h"



/*
This is example code to driver a p3(2121)64*32 -style RGB LED display. These types of displays do not have memory and need to be refreshed
continuously. The display has 2 RGB inputs, 4 inputs to select the active line, a pixel clock input, a latch enable input and an output-enable
input. The display can be seen as 2 64x16 displays consisting of the upper half and the lower half of the display. Each half has a separate
RGB pixel input, the rest of the inputs are shared.

Each display half can only show one line of RGB pixels at a time: to do this, the RGB data for the line is input by setting the RGB input pins
to the desired value for the first pixel, giving the display a clock pulse, setting the RGB input pins to the desired value for the second pixel,
giving a clock pulse, etc. Do this 64 times to clock in an entire row. The pixels will not be displayed yet: until the latch input is made high,
the display will still send out the previously clocked in line. Pulsing the latch input high will replace the displayed data with the data just
clocked in.

The 4 line select inputs select where the currently active line is displayed: when provided with a binary number (0-15), the latched pixel data
will immediately appear on this line. Note: While clocking in data for a line, the *previous* line is still displayed, and these lines should
be set to the value to reflect the position the *previous* line is supposed to be on.

Finally, the screen has an OE input, which is used to disable the LEDs when latching new data and changing the state of the line select inputs:
doing so hides any artifacts that appear at this time. The OE line is also used to dim the display by only turning it on for a limited time every
line.

All in all, an image can be displayed by 'scanning' the display, say, 100 times per second. The slowness of the human eye hides the fact that
only one line is showed at a time, and the display looks like every pixel is driven at the same time.

Now, the RGB inputs for these types of displays are digital, meaning each red, green and blue subpixel can only be on or off. This leads to a
color palette of 8 pixels, not enough to display nice pictures. To get around this, we use binary code modulation.

Binary code modulation is somewhat like PWM, but easier to implement in our case. First, we define the time we would refresh the display without
binary code modulation as the 'frame time'. For, say, a four-bit binary code modulation, the frame time is divided into 15 ticks of equal length.

We also define 4 subframes (0 to 3), defining which LEDs are on and which LEDs are off during that subframe. (Subframes are the same as a
normal frame in non-binary-coded-modulation mode, but are showed faster.)  From our (non-monochrome) input image, we take the (8-bit: bit 7
to bit 0) RGB pixel values. If the pixel values have bit 7 set, we turn the corresponding LED on in subframe 3. If they have bit 6 set,
we turn on the corresponding LED in subframe 2, if bit 5 is set subframe 1, if bit 4 is set in subframe 0.

Now, in order to (on average within a frame) turn a LED on for the time specified in the pixel value in the input data, we need to weigh the
subframes. We have 15 pixels: if we show subframe 3 for 8 of them, subframe 2 for 4 of them, subframe 1 for 2 of them and subframe 1 for 1 of
them, this 'automatically' happens. (We also distribute the subframes evenly over the ticks, which reduces flicker.)


In this code, we use the I2S peripheral in parallel mode to achieve this. Essentially, first we allocate memory for all subframes. This memory
contains a sequence of all the signals (2xRGB, line select, latch enable, output enable) that need to be sent to the display for that subframe.
Then we ask the I2S-parallel driver to set up a DMA chain so the subframes are sent out in a sequence that satisfies the requirement that
subframe x has to be sent out for (2^x) ticks. Finally, we fill the subframes with image data.

We use a frontbuffer/backbuffer technique here to make sure the display is refreshed in one go and drawing artifacts do not reach the display.
In practice, for small displays this is not really necessarily.

Finally, the binary code modulated intensity of a LED does not correspond to the intensity as seen by human eyes. To correct for that, a
luminance correction is used. See val2pwm.c for more info.

Note: Because every subframe contains one bit of grayscale information, they are also referred to as 'bitplanes' by the code below.
*/

#define DISPLAY_WIDTH  128
#define DISPLAY_HEIGHT  32

//This is the bit depth, per RGB subpixel, of the data that is sent to the display.
//The effective bit depth (in computer pixel terms) is less because of the PWM correction. With
//a bitplane count of 7, you should be able to reproduce an 16-bit image more or less faithfully, though.
#define BITPLANE_CNT 7

//64*32 RGB leds, 2 pixels per 16-bit value...
#define BITPLANE_SZ (DISPLAY_WIDTH * DISPLAY_HEIGHT / 2)


// -------------------------------------------
//  Meaning of the bits in a 16 bit DMA word
// -------------------------------------------

//Upper half RGB
#define BIT_R1 (1<<0)
#define BIT_G1 (1<<1)
#define BIT_B1 (1<<2)

//Lower half RGB
#define BIT_R2 (1<<3)
#define BIT_G2 (1<<4)
#define BIT_B2 (1<<5)

// -1 = don't care
// -1

#define BIT_A (1<<8)
#define BIT_B (1<<9)
#define BIT_C (1<<10)
#define BIT_D (1<<11)

#define BIT_LAT (1<<12)
#define BIT_OE (1<<13)

// -1
// -1

int brightness=16; //Change to set the global brightness of the display, range 1 .. DISPLAY_WIDTH - 1
                   //Warning when set too high: Do not look into LEDs with remaining eye.

uint16_t *bitplane[2][BITPLANE_CNT];
uint8_t framebuf[DISPLAY_WIDTH * DISPLAY_HEIGHT * 3];

//Get a pixel from the image at pix, assuming the image is a 64x32 8R8G8B image
//Returns it as an uint32 with the lower 24 bits containing the RGB values.
static uint32_t getPixel(uint8_t *buf, int x, int y)
{
    uint8_t *p = &buf[(x + y * DISPLAY_WIDTH) * 3];
    return (p[0] << 16) | (p[1] << 8) | p[2];
}


static void setPixel(unsigned x, unsigned y, unsigned col)
{
    uint8_t *p = &framebuf[(x + y * DISPLAY_WIDTH) * 3];
    p[0] = col >> 16;
    p[1] = col >> 8;
    p[2] = col;
}

void update_frame()
{
    static int backbuf_id=0; //which buffer is the backbuffer, as in, which one is not active so we can write to it

    for (int pl=0; pl<BITPLANE_CNT; pl++) {
        int mask=(1<<(8-BITPLANE_CNT+pl)); //bitmask for pixel data in input for this bitplane
        uint16_t *p=bitplane[backbuf_id][pl]; //bitplane location to write to
        for (unsigned int y=0; y<16; y++) {
            int lbits=0;                //Precalculate line bits of the *previous* line, which is the one we're displaying now
            if ((y-1)&1) lbits|=BIT_A;
            if ((y-1)&2) lbits|=BIT_B;
            if ((y-1)&4) lbits|=BIT_C;
            if ((y-1)&8) lbits|=BIT_D;
            for (int x=0; x<DISPLAY_WIDTH; x++) {
                int v=lbits;
                //Do not show image while the line bits are changing
                // OE is active low, it would be better to call it BLANK
                if (x >= brightness) v |= BIT_OE;

                // latch pulse at the end of shifting in row - data
                // in mode 0 (with display hack) we get natural word order, latch on DISPLAY_WIDTH - 1
                if (x == (DISPLAY_WIDTH - 1)) v |= BIT_LAT;
                // in mode 1 words are swapped, latch on DISPLAY_WIDTH - 2

                int c1, c2;
                c1=getPixel(framebuf, x, y);
                c2=getPixel(framebuf, x, y + 16);
                if (c1 & (mask<<16)) v|=BIT_R1;
                if (c1 & (mask<<8)) v|=BIT_G1;
                if (c1 & (mask<<0)) v|=BIT_B1;
                if (c2 & (mask<<16)) v|=BIT_R2;
                if (c2 & (mask<<8)) v|=BIT_G2;
                if (c2 & (mask<<0)) v|=BIT_B2;

                //Save the calculated value to the bitplane memory
                *p++=v;
            }
        }
    }
    //Show our work!
    i2s_parallel_flip_to_buffer(&I2S1, backbuf_id);
    backbuf_id ^= 1;
}

void tp_diagonal()
{
    for (unsigned y=0; y<DISPLAY_HEIGHT; y++)
        for (unsigned x=0; x<DISPLAY_WIDTH; x++)
            setPixel(x, y, (x - y) % DISPLAY_HEIGHT == 0 ? 0xFFFFFFFF : 0xFF000000);
    update_frame(framebuf);
    vTaskDelay(6000 / portTICK_PERIOD_MS);
}

void tp_stripes(unsigned width, unsigned offset, bool isY)
{
    for (unsigned y=0; y<DISPLAY_HEIGHT; y++) {
        for (unsigned x=0; x<DISPLAY_WIDTH; x++) {
            unsigned var = isY ? x : y;
            unsigned col = (var + offset) % width == 0 ? 0xFFFFFFFF : 0xFF000000;
            setPixel(x, y, col);
        }
    }
    update_frame(framebuf);
}

void tp_stripes_sequence(bool isY)
{
    for (unsigned i=0; i<8; i++) {
        printf("stripes %d / 8\n", i + 1);
        tp_stripes(8, i, isY);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    for (unsigned i=0; i<4; i++) {
        printf("stripes %d / 2\n", (i % 2) + 1);
        tp_stripes(2, i % 2, isY);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void tp_nyan(unsigned n_frames)
{
    for (unsigned i=0; i<n_frames; i++) {
        memset(framebuf, 0, sizeof(framebuf));
        //Fill bitplanes with the data for the current image
        const uint8_t *pix = &anim[(i % 12) * 64 * 32 * 3]; //pixel data for this animation frame
        for (unsigned y=0; y<32; y++) {
            for (unsigned x=0; x<64; x++) {
                const uint8_t *p = &pix[(x + y * 64) * 3];
                unsigned color = (p[0] << 16) | (p[1] << 8) | p[2];
                setPixel((x + i) % DISPLAY_WIDTH, y, color);
            }
        }
        update_frame();

        //Bitplanes are updated, new image shows now.
        vTaskDelay(50 / portTICK_PERIOD_MS); //animation has an 100ms interval
    }
}

void app_main()
{

    i2s_parallel_buffer_desc_t bufdesc[2][1<<BITPLANE_CNT];
    i2s_parallel_config_t cfg={
        // .gpio_bus={2, 15, 4, 16, 27, 17, -1, -1, 5, 18, 19, 21, 26, 25, -1, -1},
        // .gpio_clk=22,

        // -------------------
        //  Espirgbani pinout
        // -------------------
        .gpio_bus={2, 15, 13,   16, 27, 17,   -1, -1,   5, 18, 19, 23,   26, 25,   -1, -1},
        .gpio_clk=22,

        .bits=I2S_PARALLEL_BITS_16,
        // .clk_div=1,     // = 10 MHz (in mode 0 = flicker hack enabled)
        // .clk_div=2,     // = 13.3 MHz
        // .clk_div=3,     // = 10 MHz
        // .clk_div=4,     // = 8 MHz
        .clk_div=8,     // = 4.4 MHz
        // .clk_div=16,     // = 2.4 MHz

        .is_clk_inverted=true,
        .bufa=bufdesc[0],
        .bufb=bufdesc[1],
    };

    for (int i=0; i<BITPLANE_CNT; i++) {
        for (int j=0; j<2; j++) {
            bitplane[j][i]=heap_caps_malloc(BITPLANE_SZ*2, MALLOC_CAP_DMA);
            assert(bitplane[j][i] && "Can't allocate bitplane memory");
        }
    }

    //Do binary time division setup. Essentially, we need n of plane 0, 2n of plane 1, 4n of plane 2 etc, but that
    //needs to be divided evenly over time to stop flicker from happening. This little bit of code tries to do that
    //more-or-less elegantly.
    int times[BITPLANE_CNT]={0};
    printf("Bitplane order: ");
    for (int i=0; i<((1<<BITPLANE_CNT)-1); i++) {
        int ch=0;
        //Find plane that needs insertion the most
        for (int j=0; j<BITPLANE_CNT; j++) {
            if (times[j]<=times[ch]) ch=j;
        }
        printf("%d ", ch);
        //Insert the plane
        for (int j=0; j<2; j++) {
            bufdesc[j][i].memory=bitplane[j][ch];
            bufdesc[j][i].size=BITPLANE_SZ*2;
        }
        //Magic to make sure we choose this bitplane an appropriate time later next time
        times[ch]+=(1<<(BITPLANE_CNT-ch));
    }
    printf("\n");

    //End markers
    bufdesc[0][((1<<BITPLANE_CNT)-1)].memory=NULL;
    bufdesc[1][((1<<BITPLANE_CNT)-1)].memory=NULL;

    //Setup I2S
    i2s_parallel_setup(&I2S1, &cfg);

    printf("I2S setup done.\n");

    while(1) {
        tp_diagonal();
        tp_stripes_sequence(false);
        tp_stripes_sequence(true);
        tp_nyan(300);
    }
}



