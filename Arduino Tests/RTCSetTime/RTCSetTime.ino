// Date and time functions using a RX8025 RTC connected via I2C and Wire lib

#include <Wire.h>
#include "Sodaq_DS3231.h"

char weekDay[][4] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

//year, month, date, hour, min, sec and week-day(starts from 0 and goes to 6)
//writing any non-existent time-data may interfere with normal operation of the RTC.
//Take care of week-day also.
DateTime dt(2016, 4, 11, 23, 56, 0, 1);

const int resetButton = 2;

void setup () 
{
    Serial.begin(115200);
    Wire.begin();
    rtc.begin();
    pinMode(resetButton, INPUT);
    Serial.print("Waiting signal from pin ");
    Serial.println(resetButton);
    while(digitalRead(resetButton) == LOW)
      delay(1);
    Serial.println("Setting date time now!");
    rtc.setDateTime(dt); //Adjust date-time as defined 'dt' above 
}

void loop () 
{
    DateTime now = rtc.now(); //get the current date-time
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.date(), DEC);
    Serial.print(' ');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.print(' ');
    Serial.print(weekDay[now.dayOfWeek()]);
    Serial.println();
    delay(1000);
}
