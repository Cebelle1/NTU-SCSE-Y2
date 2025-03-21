#include <stdio.h>
#include <fcntl.h> // For O_RDWR
#include <unistd.h> // For close(), usleep()
#include <stdlib.h> // For EXIT_SUCCESS
#include <sys/ioctl.h> // For ioctl()
#include <linux/gpio.h> // For GPIO-related structures and ioctl commands

int main(int argc, char *argv[])
{
    int fd0 = open("/dev/gpiochip0", O_RDWR); // Open the file descriptor
    if (fd0 < 0) {
        perror("Failed to open /dev/gpiochip0");
        return EXIT_FAILURE;
    }

    struct gpiochip_info cinfo;
    if (ioctl(fd0, GPIO_GET_CHIPINFO_IOCTL, &cinfo) < 0) {
        perror("Failed to get chip info");
        close(fd0);
        return EXIT_FAILURE;
    }
    fprintf(stdout, "GPIO chip 0: %s, \"%s\", %u lines\n", cinfo.name, cinfo.label, cinfo.lines);

    struct gpiohandle_request req_GY; // Green and Yellow
    struct gpiohandle_data data_GY; // For data bit

    req_GY.lines = 2; // 2 pins in this handler
    req_GY.lineoffsets[0] = 4; // Pin 4 - Green LED
    req_GY.lineoffsets[1] = 17; // Pin 17 - Yellow LED
    req_GY.flags = GPIOHANDLE_REQUEST_OUTPUT; // Set them to be output
    data_GY.values[0] = 1; // Set initial value of Green LED to High (ON)
    data_GY.values[1] = 0; // Set initial value of Yellow LED to Low (OFF)

    if (ioctl(fd0, GPIO_GET_LINEHANDLE_IOCTL, &req_GY) < 0) {
        perror("Failed to get line handle");
        close(fd0);
        return EXIT_FAILURE;
    }

    for (int i = 0; i < 5; ++i) {
        if (ioctl(req_GY.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data_GY) < 0) {
            perror("Failed to set line values");
            close(req_GY.fd);
            close(fd0);
            return EXIT_FAILURE;
        }
        usleep(1000000); // Sleep for 1 second
        data_GY.values[0] = !data_GY.values[0]; // Toggle
        data_GY.values[1] = !data_GY.values[1]; // This should be data_GY.values[1] = !data_GY.values[1];
    }

    close(req_GY.fd); // Release line
    close(fd0); // Close the file descriptor
    return EXIT_SUCCESS;
}
