#include <hidboot.h>
#include <usbhub.h>
#include <Wire.h>
#include "MPU3050lib.h"
#include "ADXL345lib.h"
#include <InkShieldLite.h>

// Satisfy IDE, which only needs to see the include statment in the ino.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

// The image is 48 x 48 pixels, where each pixel is 1 bit.
// The image array uses 16 bit integers, hence 3 integers
// will be sufficient to represent width. Since the height is
// also 48 bits, we will need 48 integers vertically, making
// up 3 * 48 integers.
//
// The printer has 12 nozzles, hence we will index the array
// and create a 16 bit word with 12 bits filled from the image
// and rest of the bits padded with 0.
//
// Some geometry can be applied to the direction of the print
// nozzle array and corresponding bits can be extracted from
// the array to match the angle of the printer.
//
// In the first version, we will assume that the printer has
// correct orientation and not at an angle from the image array.
//
// The sensor input shall be scaled and matched to the array
// index.

const uint8_t IMAGE_WIDTH = 3;
const uint8_t IMAGE_HEIGHT = 48;
const uint8_t IMAGE_PACKET = 16;
const uint8_t NOZZLE_CNT = 12;

uint16_t CACHE[IMAGE_WIDTH * IMAGE_HEIGHT];
const uint16_t SQUARES[IMAGE_WIDTH * IMAGE_HEIGHT] = {
    0b0000000000000000, 0b0000000000000000, 0b0000000000000000,
    0b0000000000000000, 0b0000000000000000, 0b0000000000000000,
    0b0000000000000000, 0b0000000000000000, 0b0000000000000000,
    0b0000000000000000, 0b0000000000000000, 0b0000000000000000,
    0b1111111100000000, 0b1111111111111111, 0b0000000011111111,
    0b0000000100000000, 0b0000000000000000, 0b0000000010000000,
    0b0000000100000000, 0b0000000000000000, 0b0000000010000000,
    0b0000000100000000, 0b0000000000000000, 0b0000000010000000,
    0b0000000100000000, 0b0000000000000000, 0b0000000010000000,
    0b0000000100000000, 0b0000000000000000, 0b0000000010000000,
    0b0000000100000000, 0b0000000000000000, 0b0000000010000000,
    0b0000000100000000, 0b0000000000000000, 0b0000000010000000,
    0b0000000100000000, 0b1111111111111111, 0b0000000010000000,
    0b0000000100000000, 0b1000000000000001, 0b0000000010000000,
    0b0000000100000000, 0b1000000000000001, 0b0000000010000000,
    0b0000000100000000, 0b1000000000000001, 0b0000000010000000,
    0b0000000100000000, 0b1000000000000001, 0b0000000010000000,
    0b0000000100000000, 0b1000000000000001, 0b0000000010000000,
    0b0000000100000000, 0b1000000000000001, 0b0000000010000000,
    0b0000000100000000, 0b1000000000000001, 0b0000000010000000,
    0b0000000100000000, 0b1000000000000001, 0b0000000010000000,
    0b0000000100000000, 0b1000000000000001, 0b0000000010000000,
    0b0000000100000000, 0b1000000000000001, 0b0000000010000000,
    0b0000000100000000, 0b1000000000000001, 0b0000000010000000,
    0b0000000100000000, 0b1000000000000001, 0b0000000010000000,
    0b0000000100000000, 0b1000000000000001, 0b0000000010000000,
    0b0000000100000000, 0b1000000000000001, 0b0000000010000000,
    0b0000000100000000, 0b1000000000000001, 0b0000000010000000,
    0b0000000100000000, 0b1000000000000001, 0b0000000010000000,
    0b0000000100000000, 0b1000000000000001, 0b0000000010000000,
    0b0000000100000000, 0b1000000000000001, 0b0000000010000000,
    0b0000000100000000, 0b1000000000000001, 0b0000000010000000,
    0b0000000100000000, 0b1000000000000001, 0b0000000010000000,
    0b0000000100000000, 0b1000000000000001, 0b0000000010000000,
    0b0000000100000000, 0b1000000000000001, 0b0000000010000000,
    0b0000000100000000, 0b1000000000000001, 0b0000000010000000,
    0b0000000100000000, 0b1111111111111111, 0b0000000010000000,
    0b0000000100000000, 0b0000000000000000, 0b0000000010000000,
    0b0000000100000000, 0b0000000000000000, 0b0000000010000000,
    0b0000000100000000, 0b0000000000000000, 0b0000000010000000,
    0b0000000100000000, 0b0000000000000000, 0b0000000010000000,
    0b0000000100000000, 0b0000000000000000, 0b0000000010000000,
    0b0000000100000000, 0b0000000000000000, 0b0000000010000000,
    0b1111111100000000, 0b1111111111111111, 0b0000000011111111,
    0b0000000000000000, 0b0000000000000000, 0b0000000000000000,
    0b0000000000000000, 0b0000000000000000, 0b0000000000000000,
    0b0000000000000000, 0b0000000000000000, 0b0000000000000000,
    0b0000000000000000, 0b0000000000000000, 0b0000000000000000
};

class Image {
public:
    Image(const uint16_t* d, uint16_t* c,
    uint8_t w, uint8_t h, uint8_t p, uint8_t n)
:
        _data(d), _cache(c),
        _width(w), _height(h), _packet(p), _nozzles(n) {
        }
    ~Image() {
    }

    // y is 1:1, x needs to scale by packet
    bool GetBit(int32_t x, int32_t y) const;
    uint16_t GetStrip(int32_t x, int32_t y) const;
    void CacheData();

    // Dump info
    void PrintCache() const;
    void PrintBits() const;
    void PrintStrip(int32_t x, int32_t y) const;
    void PrintStrips() const;

private:
    uint16_t GetPacketIdx(uint8_t w, uint8_t h) const;

private:
    const uint16_t*     _data;
    mutable uint16_t*   _cache;
    uint8_t             _width;
    uint8_t             _height;
    const uint8_t       _packet;
    const uint8_t       _nozzles;
};

uint16_t Image::GetPacketIdx(uint8_t w, uint8_t h) const
{
    return (h * _width + w);
}

void Image::CacheData()
{
    for (uint8_t h = 0; h < _height; ++h) {
        for (uint8_t w = 0; w < _width; ++w) {
            _cache[GetPacketIdx(w, h)] = _data[GetPacketIdx(w, h)];
        }
    }
}

// y is 1:1, x needs to scale by packet
bool Image::GetBit(int32_t x, int32_t y) const
{
    if (x < 0 || y < 0 || x >= IMAGE_WIDTH * IMAGE_PACKET || y >= IMAGE_HEIGHT)
        return false;
    uint8_t w = x / _packet;
    uint8_t r = x % _packet;

    uint8_t idx = GetPacketIdx(w, y);
    uint16_t packet = _cache[idx];
    uint16_t mask = (1 << r);
    uint16_t data = packet & mask;
    _cache[idx] &= ~data; // Clear used data

        return data != 0;
}

uint16_t Image::GetStrip(int32_t x, int32_t y) const
{
    uint16_t strip = 0;
    for (uint8_t n = 0; n < _nozzles; ++n) {
        bool b = GetBit(x, y + n);
        // In order
        //strip |= (b << n);
        // Reverse
        strip |= (b << (_nozzles - n - 1));
    }
    return strip;
}

void Image::PrintCache() const
{
    for (uint8_t h = 0; h < _height; ++h) {
        for (uint8_t w = 0; w < _width; ++w) {
            uint16_t val = _cache[GetPacketIdx(w, h)];
            uint16_t mask = 1;
            for (uint8_t b = 0; b < _packet; ++b) {
                if (val & mask) {
                    Serial.print(1);
                }
                else {
                    Serial.print(0);
                }
                mask <<= 1;
            }
            Serial.print(" ");
        }
        Serial.println("");
    }
}

void Image::PrintBits() const
{
    for (uint8_t y = 0; y < IMAGE_HEIGHT; ++y) {
        for (uint8_t x = 0; x < IMAGE_WIDTH * IMAGE_PACKET; ++x) {
            Serial.print(GetBit(x, y));
        }
        Serial.println("");
    }
}

void Image::PrintStrips() const
{
    for (uint8_t y = 0; y < IMAGE_HEIGHT; y += _nozzles) {
        for (uint8_t x = 0; x < IMAGE_WIDTH * IMAGE_PACKET; ++x) {
            PrintStrip(x, y);
        }
    }
}

void Image::PrintStrip(int32_t x, int32_t y) const
{
    Serial.print("(");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(")");
    Serial.print(": ");
    uint16_t val = GetStrip(x, y);

    uint16_t mask = 1;
    for (uint8_t b = 0; b < _packet; ++b) {
        if (val & mask) {
            Serial.print(1);
        }
        else {
            Serial.print(0);
        }
        mask <<= 1;
    }
    Serial.println("");
}

/*
Mouse: Rated 1600 dpi
 Measured:
 10 inch
 x = 18838
 18725
 19032
 19057 -> 18913 = 1891.3 dpi -> Map 19.69
 y = 19950
 19437
 19862
 19754 -> 19750 = 1975 dpi -> Map 20.572
 
 Ink:
 Resolution: 96 dpi
 Nozzle: 12
 Print Swath: 0.125 in
 */
#define X_SCALE 19.69
#define Y_SCALE 21.00

// Mouse controller
class MouseRptParser : 
public MouseReportParser
{
public:
    MouseRptParser() : 
    _x(0), _y(0), _canPrint(false) {
    }
    ~MouseRptParser() {
    }
    void print();
    int32_t GetX() { 
        return _x / X_SCALE; 
    }
    int32_t GetY() { 
        return _y / Y_SCALE; 
    }
    bool canPrint() const {
        return _canPrint;
    }
    bool resetImage() const {
        return _resetImage;
    }
protected:
    virtual void OnMouseMove	        (MOUSEINFO *mi);
    virtual void OnLeftButtonUp	        (MOUSEINFO *mi);
    virtual void OnLeftButtonDown	(MOUSEINFO *mi);
    virtual void OnRightButtonUp	(MOUSEINFO *mi);
    virtual void OnRightButtonDown	(MOUSEINFO *mi);
    virtual void OnMiddleButtonUp	(MOUSEINFO *mi);
    virtual void OnMiddleButtonDown	(MOUSEINFO *mi);
private: // Data
    int32_t    _x;
    int32_t    _y;
    bool       _canPrint;
    bool       _resetImage;
};
void MouseRptParser::print()
{
    Serial.print("mouse [");
    Serial.print(GetX(), DEC);
    Serial.print(", ");
    Serial.print(GetY(), DEC);
    Serial.print("]");
}
void MouseRptParser::OnMouseMove(MOUSEINFO *mi)
{
    _x += mi->dX;
    _y += mi->dY;
};
void MouseRptParser::OnLeftButtonUp	(MOUSEINFO *mi)
{
    Serial.println("L Butt Up");
};
void MouseRptParser::OnLeftButtonDown	(MOUSEINFO *mi)
{
    Serial.println("L Butt Dn");
    _x = 0;
    _y = 0;
};
void MouseRptParser::OnRightButtonUp	(MOUSEINFO *mi)
{
    Serial.println("R Butt Up");
};
void MouseRptParser::OnRightButtonDown	(MOUSEINFO *mi)
{
    Serial.println("R Butt Dn");
    _canPrint = false;
};
void MouseRptParser::OnMiddleButtonUp	(MOUSEINFO *mi)
{
    Serial.println("M Butt Up");
};
void MouseRptParser::OnMiddleButtonDown	(MOUSEINFO *mi)
{
    Serial.println("M Butt Dn");
    _resetImage = true;
    _canPrint = true;
    _x = 0;
    _y = 0;
};

class Controller
{
public:
    Controller() {
    };
    ~Controller() {
    };
    void setup() {
    };
    void loop() {
    };

protected:
    uint32_t _curr_time;
};

class MouseController : 
public Controller
{
public:
    MouseController() : 
    _usb(), _hub(&_usb), _hidMouse(&_usb){
    };
    ~MouseController() {
    };
    void setup();
    void loop();
    void print() {
        _prs.print();
    };
    int32_t GetX() { 
        return _prs.GetX(); 
    }
    int32_t GetY() { 
        return _prs.GetY(); 
    }
    bool canPrint() const {
        return _prs.canPrint();
    }
    bool resetImage() const {
        return _prs.resetImage();
    }

private:
    USB                            _usb;
    USBHub                         _hub;
    HIDBoot<HID_PROTOCOL_MOUSE>    _hidMouse;
    MouseRptParser                 _prs;
};

void MouseController::setup()
{
    if (_usb.Init() == -1)
        Serial.println("OSC did not start.");

    delay( 200 );

    _hidMouse.SetReportParser(0,(HIDReportParser*)&_prs);
}
void MouseController::loop()
{
    _usb.Task();
}

class GyroController : 
public Controller
{
public:
    GyroController() {
    };
    ~GyroController() {
    };
    void setup();
    void loop();
    void print();

private:
    Gyroscope   _gyro;
    bool        _fail;
    uint32_t    _prev_time;
    double      _x;
    double      _y;
    double      _z;
};

void GyroController::setup()
{
    if (_gyro.begin(OSEPP_GYRO_SW_ON) != 0)
    {
        Serial.println("Error connecting to gyroscope");
        _fail = true;
        return;
    }

    _gyro.setRange(MPU3050_RANGE_PM2000);
    _curr_time = millis();
    _prev_time = _curr_time;
}
void GyroController::loop()
{
    _curr_time = millis();
    if (_curr_time - _prev_time < 50) 
        return;

    _prev_time = _curr_time;

    // don't bother reading if we failed to connect
    if (_fail)
        return;

    // Read degrees per second
    // Note: You can also read the raw data by calling
    //       readRaw() - readRaw takes int16_t instead of doubles
    if (_gyro.readDegPerSecond(&_x, &_y, &_z) != 0)
    {
        Serial.println("Failed to read gyroscope");
        return;
    }
}
void GyroController::print()
{
    // print them out
    Serial.print(" gyro [");
    Serial.print(_x);
    Serial.print(", ");
    Serial.print(_y);
    Serial.print(", ");
    Serial.print(_z);
    Serial.print("]");
}

class AccController : 
public Controller
{
public:
    AccController() {
    };
    ~AccController() {
    };
    void setup();
    void loop();
    void print();

private:
    Accelerometer   _acc;
    bool        _fail;
    uint32_t    _prev_time;
    double      _x;
    double      _y;
    double      _z;
};
void AccController::setup()
{
    if (_acc.begin(OSEPP_ACC_SW_ON) != 0)
    {
        Serial.println("Error connecting to accelerometer");
        _fail = true;
        return;
    }

    _acc.setSensitivity(ADXL345_RANGE_PM4G);
    _curr_time = millis();
    _prev_time = _curr_time;
}
void AccController::loop()
{
    _curr_time = millis();
    if (_curr_time - _prev_time < 50) 
        return;

    _prev_time = _curr_time;

    // don't bother reading if we failed to connect
    if (_fail)
        return;

    // Read Gs
    // Note: You can also read the raw data by calling
    //       readRaw() - readRaw takes int16_t instead of doubles
    if (_acc.readGs(&_x, &_y, &_z) != 0)
    {
        Serial.println("Failed to read accelerometer");
        return;
    }
}
void AccController::print()
{
    // print them out
    Serial.print(" acc [");
    Serial.print(_x);
    Serial.print(", ");
    Serial.print(_y);
    Serial.print(", ");
    Serial.print(_z);
    Serial.print("]");
}

//initialize shield on pin 2
const byte pulsePin = 2;

class InkController : 
public Controller
{
public:
    InkController() : 
    _enable(false) {
    }
    ~InkController() {
    }
    void setup();
    void loop();
    void print();
    void sprayInk(word strip);
    void canPrint(bool state) {
        _enable = state;
    }
private:
    uint32_t    _nozzle_time;
    uint32_t    _strip_time;
    bool        _enable;
};
void InkController::setup()
{
    setABCDPinMode(abcdA0A3, OUTPUT);  //set the abcd pins as outputs
    pinMode(pulsePin, OUTPUT);         //set the pulse pin as output
    _curr_time = micros();
    _strip_time = _curr_time;
    _nozzle_time = _curr_time;
}
void InkController::loop()
{
    //spray all 12 nozzles as fast as possible
    //(blackout pattern 0x0FFF = 0000111111111111)
    sprayInk(0x0FFF);
    //or other patterns
    //(every other nozzle 0x0AAA = 0000101010101010)
    //sprayInk(0x0AAA);
    //(every other nozzle 0x0555 = 0000010101010101)
    //sprayInk(0x0555);
}
void InkController::print()
{
}
void InkController::sprayInk(word strip)
{
    if (! _enable)
        return;
    _curr_time = micros();
    // wait to be sure we don't try to fire nozzles too fast and burn them out
    if (_curr_time - _strip_time < 800) 
        return;

    _strip_time = _curr_time;

    // loop thru the strip
    for(byte i = 0; i <= 11; i++){
        if(strip & 1<<i){
            fastABCDDigitalWrite(abcdA0A3, i, HIGH);  //set abcd (nozzle address)
            fastDigitalWrite(pulsePin, HIGH); 
            delayMicroseconds(5);  //pulse pin high, wait 5us
            fastDigitalWrite(pulsePin, LOW); //pulse pin low
            fastABCDDigitalWrite(abcdA0A3, i, LOW); //reset abcd
        }
    }	
}

void setupSerial()
{
    Serial.begin( 115200 );
    // Wait for serial port to connect - used on Leonardo, Teensy and other boards with 
    // built-in USB CDC serial connection
    while (! Serial);
    Serial.println("Start serial");
}

MouseController mc;
//GyroController gc;
//AccController ac;
InkController ic;
Image img(SQUARES, CACHE, IMAGE_WIDTH, IMAGE_HEIGHT, IMAGE_PACKET, NOZZLE_CNT);

void setup()
{
    setupSerial();
    mc.setup();
    //gc.setup();
    //ac.setup();
    ic.setup();
}

void loop()
{
    mc.loop();
    //gc.loop();
    //ac.loop();
    //ic.loop();
    if (mc.canPrint()) {
        uint32_t x = mc.GetX();
        uint32_t y = mc.GetY();
        uint16_t strip = img.GetStrip(x, y);
        ic.sprayInk(strip);
        mc.print();
    }


    ic.canPrint(mc.canPrint());
    if (mc.resetImage()) {
        img.CacheData();
    }

    Serial.println("");
}




























