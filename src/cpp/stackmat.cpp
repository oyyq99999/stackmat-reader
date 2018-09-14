#include "stackmat.hpp"
#include <math.h>
#include <iostream>

stackmatReader::stackmatReader(float sample_rate)
    : n_sample_baud(sample_rate / 1200), agc_factor(n_sample_baud), delay_array_length(n_sample_baud / 6 + 1) {
    delay_array_idx = 0;
    len_voltage_keep = 0;
    last_sgn = 0;
    last_power = 1;
    status = 0;
    stackmat_data_length = 0;
    bit_idle_val = 0;
    last_sgn_keep = 0;
}

int stackmatReader::proc_signal(float *signals, int length) {
    for (int i = 0; i < length; ++i) {
//        std::cout << signals[i] << ", ";
        proc_signal(signals[i]);
    }
//    std::cout << std::endl;
    int ret = status;
    status &= ~STACKMAT_TIME_UPDATED;
    return ret;
}

int stackmatReader::proc_signal(float signal) {
    last_power = last_power + (signal * signal - last_power) * agc_factor;
    if (last_power < 0.0001) {
        last_power = 0.0001;
    }
    signal = signal / sqrt(last_power);

    // Schmidt trigger
    int isEdge = (delay_array[delay_array_idx] - signal) * (last_sgn ? 1 : -1) > THRESHOLD_EDGE
                 && signal * (last_sgn ? -1 : 1) > THRESHOLD_SCHM
                 && len_voltage_keep > n_sample_baud * 0.6;

    delay_array[delay_array_idx] = signal;
    delay_array_idx = (delay_array_idx + 1) % delay_array_length;

    if (isEdge) {
        for (int i = 0; i < round(len_voltage_keep / n_sample_baud); i++) {
            rs232_decode(last_sgn);
        }
        last_sgn ^= 1;
        len_voltage_keep = 0;
    } else if (len_voltage_keep > n_sample_baud * 2) {
        rs232_decode(last_sgn);
        len_voltage_keep -= n_sample_baud;
    }
    len_voltage_keep++;

    int ret = status;
    status &= ~STACKMAT_TIME_UPDATED;
    return ret;
}

int stackmatReader::get_time() {
    return time;
}

int stackmatReader::get_status() {
    return status;
}

void stackmatReader::rs232_decode(int bit) {
    if (bit == (next_data >> 9 & 1)) {
        last_sgn_keep++;
    } else {
        last_sgn_keep = 1;
    }
    next_data = (next_data >> 1 & 0x1ff) | bit << 9;

    //timer is off
    if (last_sgn_keep > 100) {
        update_status(0, 0);
    }

    //idle
    if ((next_data & 0x3ff) == 0 || (next_data & 0x3ff) == 0x3ff) {
        bit_idle_val = -1 * bit;
        stackmat_data_length = 0;
    }

    //valid rs232 byte
    if (((next_data ^ ~bit_idle_val) & 0x201) == 0x200) {
        stackmat_data[stackmat_data_length++] = ((next_data ^ ~bit_idle_val) >> 1) & 0xff;
        next_data = bit_idle_val;
    }

    if (stackmat_data_length == 9) {
        int check_sum = 64 - 48 * 5 + stackmat_data[1] + stackmat_data[2] + stackmat_data[3] + stackmat_data[4] + stackmat_data[5];
        if (check_sum == stackmat_data[6]) {
            stackmat_data_length = 0;
            update_status(
                stackmat_data[0],
                (stackmat_data[1] - '0') * 60000 +
                (stackmat_data[2] - '0') * 10000 +
                (stackmat_data[3] - '0') *  1000 +
                (stackmat_data[4] - '0') *   100 +
                (stackmat_data[5] - '0') *    10
            );
        }
    } else if (stackmat_data_length == 10) {
        int check_sum = 64 - 48 * 6 + stackmat_data[1] + stackmat_data[2] + stackmat_data[3] + stackmat_data[4] + stackmat_data[5] + stackmat_data[6];
        if (check_sum == stackmat_data[7]) {
            update_status(
                stackmat_data[0],
                (stackmat_data[1] - '0') * 60000 +
                (stackmat_data[2] - '0') * 10000 +
                (stackmat_data[3] - '0') *  1000 +
                (stackmat_data[4] - '0') *   100 +
                (stackmat_data[5] - '0') *    10 +
                (stackmat_data[6] - '0') *     1
            );
        }
        stackmat_data_length = 0;
    }
}

void stackmatReader::update_status(char header, int time_milli) {
    int new_status = header & 0xff;

    if (header != 0) {
        new_status |= STACKMAT_ON;
    }
    if (header == 'A') {
        new_status |= STACKMAT_GREEN_LIGHT;
    }
    if (header == 'L' || header == 'A' || header == 'C') {
        new_status |= STACKMAT_LEFT_HAND;
    }
    if (header == 'R' || header == 'A' || header == 'C') {
        new_status |= STACKMAT_RIGHT_HAND;
    }
    if ((header != 'S' || (status & 0xff) == 'S') && (header == ' ' || time_milli > time)) {
        new_status |= STACKMAT_RUNNING;
    }
    if (new_status != status || time_milli != time) {
        new_status |= STACKMAT_TIME_UPDATED;
    }
    status = new_status;
    time = time_milli;
}
