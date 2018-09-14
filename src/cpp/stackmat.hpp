#ifndef STACKMAT_READER
#define STACKMAT_READER

class stackmatReader {
public:
    stackmatReader(float sample_rate);
    int proc_signal(float *signals, int length);
    int proc_signal(float signal);
    int get_time();
    int get_status();
    static const int STACKMAT_ON = 1 << 8;
    static const int STACKMAT_GREEN_LIGHT = 1 << 9;
    static const int STACKMAT_LEFT_HAND = 1 << 10;
    static const int STACKMAT_RIGHT_HAND = 1 << 11;
    static const int STACKMAT_RUNNING = 1 << 12;
    static const int STACKMAT_TIME_UPDATED = 1 << 31;
private:
    void rs232_decode(int bit);
    void update_status(char header, int time_milli);

    float delay_array[16];
    float last_power;

    int delay_array_idx;
    int len_voltage_keep;
    int last_sgn;
    int last_sgn_keep;

    char stackmat_data[11];
    int stackmat_data_length;
    int next_data;
    int bit_idle_val;

    int status;
    int time;

    const float n_sample_baud;
    const float agc_factor;
    const int delay_array_length;

    constexpr static const float THRESHOLD_SCHM = 0.2;
    constexpr static const float THRESHOLD_EDGE = 0.7;
};

#endif
