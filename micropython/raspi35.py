#import webrepl_setup   #setup file transfer
# access by web, need login MicroPython accesspoint, pw is micropythoN
#exec(open("raspi35.py").read())

from machine import Pin, SPI
from time import sleep

#define TFT_CS   PIN_D8  // Chip select control pin D8
#define TFT_DC   PIN_D3  // Data Command control pin
#define TFT_RST  PIN_D4  // Reset pin (could connect to NodeMCU RST, see next line)
#define TOUCH_CS PIN_D1     // Chip select pin (T_CS) of touch screen

TFT_CS = 15   # TFT Chip select control pin D8
TFT_DC = 0    # Data Command control pin D3
TFT_RST = 2   # Reset pin pin D4
TOUCH_CS = 5  # Touch Chip select control pin D1

addr_col = 0x0FFF
addr_row = 0x0FFF

#define TFT_NOP     0x00
#define TFT_SWRST   0x01

#define TFT_CASET   0x2A
#define TFT_PASET   0x2B
#define TFT_RAMWR   0x2C

#define TFT_RAMRD   0x2E
#define TFT_IDXRD   0x00 // ILI9341 only, indexed control register read

#define TFT_MADCTL  0x36
#define TFT_MAD_MY  0x80
#define TFT_MAD_MX  0x40
#define TFT_MAD_MV  0x20
#define TFT_MAD_ML  0x10
#define TFT_MAD_BGR 0x08
#define TFT_MAD_MH  0x04
#define TFT_MAD_SS  0x02
#define TFT_MAD_GS  0x01
#define TFT_MAD_RGB 0x00

#define TFT_INVOFF  0x20
#define TFT_INVON   0x21


#TFT_NOP=     0x00
#TFT_SWRST=   0x01

TFT_CASET=   0x2A
TFT_PASET=   0x2B
TFT_RAMWR=   0x2C

#TFT_RAMRD=   0x2E
#TFT_IDXRD=   0x00 #// ILI9341 only, indexed control register read
#TFT_MADCTL=  0x36
#TFT_MAD_MY=  0x80
#TFT_MAD_MX=  0x40
#TFT_MAD_MV=  0x20
#TFT_MAD_ML=  0x10
#TFT_MAD_BGR= 0x08
#TFT_MAD_MH=  0x04
#TFT_MAD_SS=  0x02
#TFT_MAD_GS=  0x01
#TFT_MAD_RGB= 0x00

#TFT_INVOFF=  0x20
#TFT_INVON=   0x21

p_cs = Pin(TFT_CS, Pin.OUT)    # create output pin on TFT_CS
p_dc = Pin(TFT_DC, Pin.OUT)    # create output pin on TFT_CS
p_rst = Pin(TFT_RST, Pin.OUT)    # create output pin on TFT_CS
p_touch_cs = Pin(TOUCH_CS, Pin.OUT)    # create output pin on TFT_CS
spi = SPI(1, baudrate=20000000, polarity=0, phase=0)

def DC_C():
    p_dc.value(0)
    
def DC_D():
    p_dc.value(1)

def CS_L():
    p_cs.value(0)

def CS_H():
    p_cs.value(1)

def tft_Write_8(C):
    buf = bytearray(2)
    buf[0] = 0
    buf[1] = C
    spi.write(buf)

def tft_Write_16(C):
    buf = bytearray(2)
    buf[0] = (C >> 8) & 0xff
    buf[1] = C & 0xff
    spi.write(buf)

def writecommand(c):
    DC_C();
    CS_L();
    tft_Write_8(c);
    CS_H();
    DC_D();

def writedata(d):
    CS_L();
    tft_Write_8(d);
    CS_H();

def tft_init():
    p_rst.value(0)
    sleep(0.1)
    p_rst.value(1)
    
    spi.init()

    # From https://github.com/notro/fbtft/blob/master/fb_ili9486.c
    #writecommand(0x01); # SW reset
    #delay(120);
	
    writecommand(0x11); # Sleep out, also SW reset
    sleep(0.12);

    writecommand(0x3A);
    writedata(0x55);
 
    writecommand(0xC2);
    writedata(0x44);

    writecommand(0xC5);
    writedata(0x00);
    writedata(0x00);
    writedata(0x00);
    writedata(0x00);

    writecommand(0xE0);
    writedata(0x0F);
    writedata(0x1F);
    writedata(0x1C);
    writedata(0x0C);
    writedata(0x0F);
    writedata(0x08);
    writedata(0x48);
    writedata(0x98);
    writedata(0x37);
    writedata(0x0A);
    writedata(0x13);
    writedata(0x04);
    writedata(0x11);
    writedata(0x0D);
    writedata(0x00);
 
    writecommand(0xE1);
    writedata(0x0F);
    writedata(0x32);
    writedata(0x2E);
    writedata(0x0B);
    writedata(0x0D);
    writedata(0x05);
    writedata(0x47);
    writedata(0x75);
    writedata(0x37);
    writedata(0x06);
    writedata(0x10);
    writedata(0x03);
    writedata(0x24);
    writedata(0x20);
    writedata(0x00);
 
    writecommand(0x20);                     # display inversion OFF
  
    writecommand(0x36);
    writedata(0x48);

    writecommand(0x29);                     # display on
    sleep(0.15);



def setAddrWindow(x0, y0, x1, y1):    
    #//spi_begin();
    global addr_col,addr_row

    addr_col = 0xFFFF
    addr_row = 0xFFFF
    
#// Column addr set    
    DC_C();
    CS_L();

    tft_Write_8(TFT_CASET);

    DC_D();

    tft_Write_8(x0>>8);
    tft_Write_8(x0);
    tft_Write_8(x1>>8);
    tft_Write_8(x1);

    #// Row addr set
    DC_C();
    tft_Write_8(TFT_PASET);

    DC_D();

    tft_Write_8(y0>>8);
    tft_Write_8(y0);
    tft_Write_8(y1>>8);
    tft_Write_8(y1);

    #// write to RAM
    DC_C();

    tft_Write_8(TFT_RAMWR);

    DC_D();

    #//spi_end();

def spi_begin():
    return

def spi_end():
    return


_width = 320
_height = 480

def writeBlock(color, repeat):
    r = repeat
    buf = bytearray(2)
    buf[0] = (color >> 8) & 0xff
    buf[1] = color & 0xff

    while (r > 0):
        spi.write(buf)
        r = r - 1


def fillRect(x, y, w, h, color):
    #// rudimentary clipping (drawChar w/big text requires this)
    if ((x > _width) or (y > _height) or (w < 1) or (h < 1)):
        return
    if ((x + w - 1) > _width):
        w = _width  - x
    if ((y + h - 1) > _height):
        h = _height - y

    spi_begin();
    setAddrWindow(x, y, x + w - 1, y + h - 1);
    writeBlock(color, w * h);
    CS_H();
    spi_end();

def drawPixel(x, y, color):
    global addr_col,addr_row
    setAddrWindow(x, y, 1, 1);
    writeBlock(color, 1);
    CS_H();

def drawFastHLine(x, y, w, color):
    #// Rudimentary clipping
    if ((x >= _width) or (y >= _height) or (w < 1)):
        return;
    if ((x + w - 1) >= _width):
        w = _width - x;
    spi_begin();
    setAddrWindow(x, y, x + w - 1, y);
    writeBlock(color, w);
    CS_H();
    spi_end();


def drawCircle(x0, y0, r, color):
    x  = 0;
    dx = 1;
    dy = r+r;
    p  = -(r>>1);

    #spi_begin();
    #inTransaction = True;

    #// These are ordered to minimise coordinate changes in x or y
    #// drawPixel can then send fewer bounding box commands
    drawPixel(x0 + r, y0, color);
    drawPixel(x0 - r, y0, color);
    drawPixel(x0, y0 - r, color);
    drawPixel(x0, y0 + r, color);

    while(x<r):
        if(p>=0):
            dy = dy-2;
            p = p-dy;
            r = r-1;

        dx = dx+2;
        p = p+dx;
        x = x+1;

        #// These are ordered to minimise coordinate changes in x or y
        #// drawPixel can then send fewer bounding box commands
        drawPixel(x0 + x, y0 + r, color);
        drawPixel(x0 - x, y0 + r, color);
        drawPixel(x0 - x, y0 - r, color);
        drawPixel(x0 + x, y0 - r, color);

        drawPixel(x0 + r, y0 + x, color);
        drawPixel(x0 - r, y0 + x, color);
        drawPixel(x0 - r, y0 - x, color);
        drawPixel(x0 + r, y0 - x, color);

    #inTransaction = False;
    #spi_end();

def fillCircle(x0, y0, r, color):
  x  = 0;
  dx = 1;
  dy = r+r;
  p  = -(r>>1);

  #spi_begin();
  #inTransaction = True;

  drawFastHLine(x0 - r, y0, dy+1, color);

  while(x<r):
      if(p>=0):
          dy-=2;
          p-=dy;
          r-=1;

      dx+=2;
      p+=dx;
      x+=1;

      drawFastHLine(x0 - r, y0 + x, 2 * r+1, color);
      drawFastHLine(x0 - r, y0 - x, 2 * r+1, color);
      drawFastHLine(x0 - x, y0 + r, 2 * x+1, color);
      drawFastHLine(x0 - x, y0 - r, 2 * x+1, color);

  #inTransaction = False;  
  #spi_end();


print('start init')
tft_init()
fillRect(0,0,30,30,0x780F)
drawCircle(120,240,30,0xF800)
fillCircle(120,280,30,0x07E0)
#print('TFT_PURPLE')
#fillRect(0,0,30,30,0x780F)
#sleep(1);
#print('TFT_GREEN')
#fillRect(100,80,100,150,0x07E0)
#sleep(5);
#print('TFT_RED')
#fillRect(100,200,120,80,0xF800)
print('end')


