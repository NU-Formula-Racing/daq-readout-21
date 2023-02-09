/*
#include "RTClib.h"
#include <map>

class LoggerBoard
{
    public: 
        LoggerBoard();

        void saveData();
        DateTime init_RTC();
        void init_SD(DateTime now);
    
    private:
        String FileName;
        const int CSpin = 5;
        const int new_SDA = 21;
        const int new_SCL = 22;

    /*
        std::map<std::string, float> 10ms_sensors;
        std::map<std::string, float> 100ms_sensors;
        std::map<std::string, float> 1000ms_sensors;
    
} ;
*/
