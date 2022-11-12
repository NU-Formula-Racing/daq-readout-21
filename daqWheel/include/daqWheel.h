/**
 * @brief Definitions and constants for the DAQ wheel board. Initializes with a wheel speed sensor pin, 
 * brake temp sensor pin, sus pos sensor pin, and can frame address.
 * 
 */
class WheelBoard
{
    public:
        static constexpr int wheelSpeedSensorPin = 34;
        WheelBoard();
        void ReadWheelSpeedSensorDuration();
        float ReadWheelSpeedSensor();
        float ReadBrakeTempSensor();

    private:
        // pins
        static constexpr int brakeTempSensorPin = 35;
        // scalars and offsets
        static constexpr float brakeTempScalar = 1;
        static constexpr float brakeTempOffset = 0;
        // wheel speed sensor
        unsigned long current_pulse_time;
        unsigned long previous_pulse_time;
        unsigned long pulse_duration;
};