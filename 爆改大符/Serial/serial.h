//
// Created by eski on 7/2/22.

#include <string.h>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdint>
#define ECHOFLAGS (ECHO | ECHOE | ECHOK | ECHONL)

using namespace std;

#define R_NUM 10
#define S_NUM 9

struct SendData {
    float yaw_angle;
    float pitch_angle;
    int distance;
    int detect_flag;
    float yaw_rps;
    float delta_yaw;

    void clear(){
        yaw_angle = 0.0;
        pitch_angle = 0.0;
        distance = 0;
        detect_flag = 0;
        yaw_rps = 0;
        delta_yaw = 0;
    }
};

class SerialPort{
public:
    SerialPort(){};
    ~SerialPort(){};

    /***
     *  init the serialport
     * @param uart_device_name  the name of the serial port device
     * @param serial_speed   the read or write speed baudrate
     * @param parity   the proof technique of the data    odd even
     * @param bits   the size of byte
     * @param stop   stop bit number
     * @return
    ***/
    bool initSerial(const char * uart_device_name, const int &serial_speed,
                    char parity, int bits, int stop);

    /***
     * Send the frame to lower computer
     * @param n : the size of the transmit array
     * @param pc2armframe : the struct which save the data
     * @param frame_head : frame header
     * @param frame_tail : frame tail
     * @return
     */
    bool sendData(int n, SendData send_data, unsigned char frame_head, unsigned char frame_tail);
    bool receiveData(int n, unsigned char fh, unsigned char ft, unsigned char * r_data);
    bool debugData(float * data);
    bool resetSerial();
    bool closeSerial();

private:

    int fd;
    int fail_fps = 0;
    const int reset_fps = 60;
    struct termios SerialPortSettings;
    unsigned char receive_data[50];  // which receive the seriol port read datas
    unsigned char transmit_data[50];  // which transmit the datas to seriol port

    struct SerialPortParam{
        const char * uart_device_name;  // serialport name
        int serial_speed;   // Baud rate
        char parity;    // Parity check
        int bits;   // the size of byte
        int stop;   // stop bit
    }SP_param;

    /***
     * to set the baudrate
     * @param speed 2400,4800,9600,57600,115200,460800
     * @return
    ***/
    bool setSpeed(const int &speed);

    /***
     * to set the parity stop bits
     * @param parity NO ODD EVEN
     * @param stop 2 or 1
     * @param bits CSIZE CS7 CS8
     * @return
    ***/
    bool setDataFormat(char &parity, int &stop, int &bits);

    bool setDispMode(const int &m_mode);

    bool turnOff();

};


