#include <stdio.h>
#include <stdint.h>

#include <unistd.h>
#include <fcntl.h>
#include <time.h>

#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <sys/ioctl.h>
#include <string.h>

#define AHT10_ADDR 0x38

// Delays

#define AHT10_POWERON_DELAY 40
#define AHT10_MEAS_DELAY 80
#define AHT10_CMD_DELAY 350
// Command bytes

#define AHT10_CMD_INIT 0b11100001
#define AHT10_CMD_MEAS 0b10101100
#define AHT10_CMD_RST  0b10111010

// Flags in the answer byte/command(?)

#define AHT10_RESP_ERROR 0xFF

#define AHT10_CAL_ENABLED (1u << 3u)
#define AHT10_BUSY        (1u << 7u)
#define AHT10_MODE_NOR    (0b11u << 5u)
#define AHT10_MODE_CYC    (0b01u << 5u)
#define AHT10_MODE_CMD    (0b10u << 5u)

const uint8_t cmd_init[] = {AHT10_CMD_INIT, AHT10_CAL_ENABLED | AHT10_MODE_CYC, 0x00};
const uint8_t cmd_meas[] = {AHT10_CMD_MEAS, 0x33, 0x00};
const uint8_t cmd_rst[] = {AHT10_CMD_RST, 0x00, 0x00};

typedef struct aht10_meas_t {
    unsigned char data[6];
    double humidity;
    double temperature;
} aht10_meas_t;

void mssleep(int ms) {
    struct timespec time;
    time.tv_sec = (ms / 1000);
    time.tv_nsec = (ms % 1000) * 1000000;
    nanosleep(&time, NULL);
}

int aht10_init(int file) {
    write(file, &cmd_init, 3);
    mssleep(AHT10_CMD_DELAY);
    return 0;
}

int aht10_measure(int file, aht10_meas_t *result) {
    unsigned char *data = result->data;
    unsigned char busy;

    // Write measure command
    write(file, &cmd_meas, 3);

    mssleep(AHT10_MEAS_DELAY);

    do {
        // Read the result bytes
        int bytes_read = read(file, data, 6);

        // Check busy flag
        busy = data[0] & AHT10_BUSY;
        if (bytes_read < 0) {
            printf("Error while reading from bus!\n");
            return -1;
        } else if (bytes_read < 6) {
            busy = 1;
            printf("Only read %d bytes (expected 6)", bytes_read);
        }

        // Sleep some more if necessary
        if (busy) {
            mssleep(1);
        }
    } while (busy);

    uint32_t hum_i = ((uint32_t) data[1] << 12u) | ((uint32_t) data[2] << 4u) | (data[3] & 0xF0u >> 4u);
    uint32_t temp_i = ((uint32_t) (data[3] & 0x0Fu) << 16u) | ((uint32_t) data[4] << 8u) | data[5];

#define divisor 1048576.0 // (= 2^20)

    double temp = ((double) temp_i * (200.0 / divisor)) - 50.0;
    double hum = (double) hum_i * (100.0 / divisor);

    result->temperature = temp;
    result->humidity = hum;
    return 0;
}

int aht10_rst(int file) {
    write(file, &cmd_rst, 1);
    return 1;
}

int main(int argc, char *argv[]) {
    int file, init, meas, measq;

    file = open("/dev/i2c-1", O_RDWR);

    if (file < 0) {
        printf("Could not open i2c device");
        return -1;
    }

    if (ioctl(file, I2C_SLAVE, AHT10_ADDR) < 0) {
        printf("Could not set slave address");
        return -2;
    }

    if (argc != 2) {
        printf("Error! Expecting operation: init, meas or measq.\n");
        return 1;
    }

    init = strcmp(argv[1], "init") == 0;
    measq = strcmp(argv[1], "measq") == 0;
    meas = measq || strcmp(argv[1], "meas") == 0;

    if (!init && !measq && !meas) {
        printf("Error! Invalid operation.\n");
        return 1;
    }

    if (init) {
        mssleep(AHT10_POWERON_DELAY);
        int initres = aht10_init(file);

        if (initres != 0) {
            printf("Failed to init.");
            return 0;
        }
    } else if (meas) {
        aht10_meas_t measurement;
        int measured = aht10_measure(file, &measurement);

        if (measured != 0) {
            printf("Failed to perform measurement!\n");
            return 0;
        }

        if (!measq) {
            printf("Data: 0x");
            for (int i = 0; i < 6; i++) {
                printf("%02X", measurement.data[i]);
            }
            printf("\n");
        }
        if (measq) {
            printf("%.02f,%.02f", measurement.temperature, measurement.humidity);
        } else {
            printf("Temperature: %.02fC\n", measurement.temperature);
            printf("Humidity: %02f%%\n", measurement.humidity);
        }
    }
    return 0;
}