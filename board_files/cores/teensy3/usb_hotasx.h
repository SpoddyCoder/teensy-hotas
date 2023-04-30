/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2013 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef USBhotasx_h_
#define USBhotasx_h_

#include "usb_desc.h"

#if defined(HOTASX_INTERFACE)

#include <inttypes.h>

// C language implementation
#ifdef __cplusplus
extern "C" {
#endif
int usb_hotasx_send(void);
extern uint32_t usb_hotasx_data[5];
#ifdef __cplusplus
}
#endif

// C++ interface
#ifdef __cplusplus
class usb_hotasx_class
{
        public:
        void begin(void) { }
        void end(void) { }
	void button(uint8_t button, bool val)
	{
		if (--button >= 32) return;
		if (val) usb_hotasx_data[0] |= (1 << button);
		else usb_hotasx_data[0] &= ~(1 << button);
		if (!manual_mode) usb_hotasx_send();
	}
	void hat(uint8_t u, uint8_t d, uint8_t l, uint8_t r)
	{
    uint32_t val;
    if ((u==1)&&(d==1)&&(l==1)&&(r==1)) val = 15;
    else if (u==0)
    {
       if (r==0) val = 1;  //UR
       else if (l==0) val = 7; //UL
       else val = 0;  //U
    }
    else if (d==0)
    {
       if (r==0) val = 3;  //UR
       else if (l==0) val = 5; //DL
       else val = 4;  //D
    }
    else if (r==0) val = 2; //R
    else val = 6; //if (l==0) //L
    usb_hotasx_data[1] = (usb_hotasx_data[1] & 0x0FFFFFFF) | (val<<28);
    if (!manual_mode) usb_hotasx_send();
  }
	void X(int invert, int val)
	{
    if (invert) val=-val;
		usb_hotasx_data[2] = (usb_hotasx_data[2] & 0xFFFFF000) | val;
		if (!manual_mode) usb_hotasx_send();
	}
	void Y(int invert, int val)
	{
    if (invert) val=-val;
		usb_hotasx_data[2] = (usb_hotasx_data[2] & 0xFF000FFF) | (val << 12);
		if (!manual_mode) usb_hotasx_send();
	}
	void Z(int invert, int val)
	{
	  if (invert) val=-val;
		usb_hotasx_data[2] = (usb_hotasx_data[2] & 0x00FFFFFF) | (val << 24);
		usb_hotasx_data[3] = (usb_hotasx_data[3] & 0xFFFFFFF0) | (val >> 8);
		if (!manual_mode) usb_hotasx_send();
	}
	void Xrotate(int invert, int val)
	{
	  if (invert) val=-val;
		usb_hotasx_data[3] = (usb_hotasx_data[3] & 0x0FFFFFFF) | (val << 28);
    usb_hotasx_data[4] = (usb_hotasx_data[4] & 0xFFFFFF00) | (val >> 4);
		if (!manual_mode) usb_hotasx_send();
	}
	void Yrotate(int invert, int val)
	{
	  if (invert) val=-val;
		usb_hotasx_data[4] = (usb_hotasx_data[4] & 0xFFF000FF) | (val << 8);
		if (!manual_mode) usb_hotasx_send();
	}
	void Zrotate(int invert, int val)
	{
	  if (invert) val=-val;
		usb_hotasx_data[3] = (usb_hotasx_data[3] & 0xFFFF000F) | (val << 4);
		if (!manual_mode) usb_hotasx_send();
	}
	void slider(int invert, unsigned char slidernum, int val)
	{
	  if (invert) val=-val;
		if (slidernum==0)
       usb_hotasx_data[3] = (usb_hotasx_data[3] & 0xF000FFFF) | (val << 16);
    else
       usb_hotasx_data[4] = (usb_hotasx_data[4] & 0x000FFFFF) | (val << 20);
		if (!manual_mode) usb_hotasx_send();
	}
	void useManualSend(bool mode)
	{
		manual_mode = mode;
	}
	void send_now(void)
	{
		usb_hotasx_send();
	}
	private:
	static uint8_t manual_mode;
};
extern usb_hotasx_class HotasX;

#endif // __cplusplus

#endif // HOTASX_INTERFACE

#endif // USBhotasx_h_

