# STM32 L86 GNSS PARSER
### STM32 GNSS Parser lib customized for model rocket avionic system

You can fint the original parser lib on [L86-GNSS-Parser-Lib](https://github.com/noumanimpra/L86-GNSS-Parser-Lib/tree/main).

<br>
<div align="center">
    <p>
    <a href="https://www.linkedin.com/in/nouman-nteli-impraim/" target="blank"><img align="center" src="https://raw.githubusercontent.com/rahuldkjain/github-profile-readme-generator/master/src/images/icons/Social/linked-in-alt.svg" alt="nouman nteli impraim" height="30" width="40" /></a>
    </p>
</div>


---
### Description

<div align="center"> 
    <p>This library aims to [parse L86 GNSS data]. 
    Designed to parse GNSS data from the Quectel L86 module, tailored for use in model rocket avionics systems. It processes the incoming NMEA sentences and extracts essential GPS information, such as latitude, longitude, altitude, and satellite count. The parser is optimized for STM32 microcontrollers but can be adapted to other platforms.
    <br>
    Current Version: v1.0.2
    However, please note that there may be missing features, bugs, or incomplete documentation.
    Contributions are welcome! ⚠️
</div>

### Usage Ex.
```C
#include "L86_GNSS_Parser.h"
extern _f g_GnssRx_Flag;
static S_GPS_L86_DATA gnss_data;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
/*
typedef struct S_GPS_L86_DATA_TAG
{
    float lat;
    float lon;
    float speedKN;
    float timeDateBuf;
    float fixedTime;
    float fixedLatBaseFormat;
    float fixedLonBaseFormat;
    int fixQualityID;
    int satInUse;
    float hdop;
    float altitudeInMeter;
    float WGS84;

}S_GPS_L86_DATA;

*/

UsrGpsL86Init(&huart2);
while(1)
{
    if (g_GnssRx_Flag)
    {
        Usr_GpsL86GetValues(&gnss_data);
    }
}

```

### Current Status
- [X] Basic functionality
- [X] Full feature set
- [ ] Complete documentation

### Contributions
Contributions, suggestions, and feedback are highly appreciated! 

---
