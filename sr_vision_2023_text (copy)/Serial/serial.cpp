//
// Created by eski on 7/2/22.
//

#include "serial.h"
#include "math.h"

bool SerialPort::initSerial(const char * uart_device_name, const int &serial_speed,
                            char parity, int bits, int stop)
{
    SP_param.uart_device_name = uart_device_name;
    SP_param.serial_speed = serial_speed;
    SP_param.parity = parity;
    SP_param.bits = bits;
    SP_param.stop = stop;
    cout << "Opening " << uart_device_name << endl;

    // O_RDWR : open in read and write
    // O_NONBLOCK : open in a non-blocking manner
    fd = open(uart_device_name, O_RDWR | O_NONBLOCK);

    if(fd == -1){
        cerr << "Opening serial fail! " << endl;
        return false;
    }
    else{
        cout << "Opening serial success! " << endl;
        tcgetattr(fd, &SerialPortSettings); // Get the current attributes of the Serial port
        setSpeed(serial_speed);
        setDataFormat(parity, stop, bits);
        setDispMode(0);
        SerialPortSettings.c_cc[VTIME] = 50; /* wait 0 seconds */
        SerialPortSettings.c_cc[VMIN] = 0;
        turnOff();//
        tcflush(fd, TCIOFLUSH);
        if (tcsetattr(fd, TCSANOW, &SerialPortSettings) != 0) {
            cerr << "Set failed!" << endl;
            return false;
        }
        else{
            cout << "fd: " << fd << endl;
            cout << "Set successful!" << endl;
            return true;
        }
    }
}

bool SerialPort::sendData(int n, SendData send_data, unsigned char frame_head, unsigned char frame_tail){
    unsigned char sum = 0;

    // 1 yaw 100 -- 2 yaw 11 -- 3 yaw 0.11
    //

    transmit_data[0] = frame_head;
    transmit_data[1] = (unsigned char) ((int)send_data.yaw_angle/100);    // integer part
    transmit_data[2] = (unsigned char) ((int)(send_data.yaw_angle)%100);  // fractional part
    transmit_data[3] = (unsigned char) ((int)(send_data.yaw_angle*100)%100);

    transmit_data[4] = (unsigned char) ((int)send_data.pitch_angle % 256);
    transmit_data[5] = (unsigned char) (((int) (send_data.pitch_angle * 100) % 100));
    transmit_data[6] = (unsigned char) (int(send_data.distance/100));   // High eight
    transmit_data[7] = (unsigned char) (send_data.detect_flag);    // detection marker

/*    transmit_data[0] = frame_head;
    transmit_data[1] = (unsigned char) ((int) send_data.yaw_angle % 256);    // integer part
    transmit_data[2] = (unsigned char) (((int) (send_data.yaw_angle * 100) % 100));  // fractional part
    transmit_data[3] = (unsigned char) ((int) send_data.pitch_angle % 256);
    transmit_data[4] = (unsigned char) (((int) (send_data.pitch_angle * 100) % 100));
    transmit_data[5] = (unsigned char) (send_data.distance >> 8);   // High eight
    transmit_data[6] = (unsigned char) (send_data.distance);    // Lower eight
    transmit_data[7] = (unsigned char) (send_data.detect_flag);    // detection marker*/

    for (int i=1; i<n-1; i++) {
        sum += transmit_data[i];
    }

    transmit_data[n-1] = (unsigned char) (sum);
#ifdef PRINT_ORIGINAL_SEND_DATA
    printf("send @ -- ");
    for (int i = 0; i <=n-1; i++) {
        printf("%d ", (int) transmit_data[i]);
    }
    printf(" -- @");
#endif
    printf("\n");
//    fd=13;
    int t_status = write(fd, transmit_data, (size_t)n);
    if (t_status < 0) {
        fail_fps ++;
        perror("Send Failed\n");
        tcflush(fd, TCOFLUSH);
        if (fail_fps > reset_fps)
            resetSerial();
        return false;
    }else{
        fail_fps = 0;
    }
    return true;
}

bool SerialPort::receiveData(int n, unsigned char fh, unsigned char ft, unsigned char * r_data) {
    bzero(receive_data, 8 * n);
    int r_status = 0;
    for (int i = 0; i < reset_fps; ++i) {
        r_status = read(fd, receive_data, sizeof(receive_data));
        if (r_status == 0) {
            cout << "Receive failed! " << endl;
            continue;
        } else if (r_status == -1) {
            resetSerial();
            continue;
        } else {
            for (int i = r_status - 1; i >= 0; --i) {
                int End = i;    // 数组结尾，帧尾
                int Begin = End - n + 1;  // 数组开头，帧头
                int Sum = End - 1;  // 和校验处于帧尾前一个，第七个字节，数组第六个元素
                if (receive_data[End] == ft && receive_data[Begin] == fh) {
                    unsigned char sum = 0;
                    for (int j = Begin + 1; j < Sum; ++j)
                        sum = sum + receive_data[j];
                    if ((unsigned char) sum != receive_data[Sum])
                        continue;
                    else {
                        for (int j = 0; j < n; ++j) {
                            r_data[j] = receive_data[Begin];
                            ++Begin;
                        }
#ifdef PRINT_ORIGINAL_RECEIVE_DATA
                        cout << "receive # -- " ;
                        for (int j=0; j<n; ++j)
                            cout << int(r_data[j]) << " " ;
                        cout <<" -- #"<< endl;
#endif
                        tcflush(fd, TCIFLUSH);
                        return true;
                    }
                }
            }
            tcflush(fd, TCIFLUSH);
            return false;
        }
    }
    //ResetSerial();
    return false;
}



bool SerialPort::setSpeed(const int &speed){
    switch (speed) {
        case 2400:
            cfsetispeed(&SerialPortSettings,B2400);
            cfsetospeed(&SerialPortSettings,B2400);
            break;
        case 4800:
            cfsetispeed(&SerialPortSettings,B4800);
            cfsetospeed(&SerialPortSettings,B4800);
            break;
        case 9600:
            cfsetispeed(&SerialPortSettings,B9600);
            cfsetospeed(&SerialPortSettings,B9600);
            break;
        case 57600:
            cfsetispeed(&SerialPortSettings,B57600);
            cfsetospeed(&SerialPortSettings,B57600);
            break;
        case 115200:
            cfsetispeed(&SerialPortSettings,B115200);
            cfsetospeed(&SerialPortSettings,B115200);
            break;
        case 460800:
            cfsetispeed(&SerialPortSettings,B460800);
            cfsetospeed(&SerialPortSettings,B460800);
            break;
        default:
            cfsetispeed(&SerialPortSettings,B9600);
            cfsetospeed(&SerialPortSettings,B9600);
            return false;
    }
    return true;
}

bool SerialPort::setDataFormat(char &parity, int &stop, int &bits){
    SerialPortSettings.c_cflag  &= ~CSIZE;
    switch(bits){
        case 7:
            SerialPortSettings.c_cflag |= CS7;
            break;
        case 8:
            SerialPortSettings.c_cflag |= CS8;
            break;
        default:
            printf("Unsupported data size\n");
            return false;
    }
    switch (stop) {
        case 1:
            SerialPortSettings.c_cflag &= ~CSTOPB;
            break;
        case 2:
            SerialPortSettings.c_cflag |= CSTOPB;
            break;
        default:
            printf("Unsupported stop bits\n");
            return false;
    }
    switch (parity) {
        case 'N':
            SerialPortSettings.c_cflag &= ~PARENB;   /* Clear parity enable */
            SerialPortSettings.c_iflag &= ~INPCK;     /* Enable parity checking */
            break;
        case 'O':
            SerialPortSettings.c_cflag |= (PARODD | PARENB); /* Enable ODD */
            SerialPortSettings.c_iflag |= INPCK;             /* Enable parity checking */
            break;
        case 'E':
            SerialPortSettings.c_cflag |= PARENB;     /* Enable parity */
            SerialPortSettings.c_cflag &= ~PARODD;    /* Enable EVEN */
            SerialPortSettings.c_iflag |= INPCK;      /* Enable parity checking */
            break;
        default:
            printf("Unsupported parity\n");
            return false;
    }
    switch (stop) {
        case 1:
            SerialPortSettings.c_cflag &= ~CSTOPB;
            break;
        case 2:
            SerialPortSettings.c_cflag |= CSTOPB;
            break;
        default:
            printf("Unsupported stop bits\n");
            return false;
    }
    return true;
}

bool SerialPort::setDispMode(const int &m_mode) {
    int status = -1;
    struct termios options{};
    if (tcgetattr(fd, &options) == -1) {
        printf("Cannot get the attribution of the terminal\n");
        return false;
    }
    if (m_mode == 1)
        options.c_lflag |= ECHOFLAGS;
    else
        options.c_lflag &= ~ECHOFLAGS;
    status = tcsetattr(fd, TCSAFLUSH, &options);
    if (status == -1 || status == EINTR) {
        printf("Cannot set the attribution of the terminal\n");
        return false;
    }
    return true;
}

bool SerialPort::turnOff(){
    SerialPortSettings.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    SerialPortSettings.c_lflag &= ~(ICANON);
    SerialPortSettings.c_lflag &= ~(ICANON | ISIG);
    SerialPortSettings.c_iflag &= ~(ICRNL | IGNCR);
    SerialPortSettings.c_lflag &= ~(ECHO | ECHOE);
    SerialPortSettings.c_oflag &= ~OPOST;
    return true;
}

bool SerialPort::resetSerial() {
    const char * uart_device_ = SP_param.uart_device_name;
    int serial_speed_ = SP_param.serial_speed;
    char parity_ = SP_param.parity;
    int bits_ = SP_param.bits;
    int stop_ = SP_param.stop;

    printf("Again Open %s ...\n", uart_device_);
    fd = open(uart_device_, O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        perror("Again Open Serial Error\n");
        return false;
    }
    tcgetattr(fd, &SerialPortSettings); // Get the current attributes of the Serial port

    setSpeed(serial_speed_);
    setDataFormat(parity_, stop_, bits_);
    setDispMode(0);

    SerialPortSettings.c_cc[VTIME] = 0; /* wait 0 seconds */
    SerialPortSettings.c_cc[VMIN] = 0;
    turnOff();//
    tcflush(fd, TCOFLUSH);
    if (tcsetattr(fd, TCSANOW, &SerialPortSettings) != 0) {
        cerr << "com set error!" << endl;
        return false;
    }
    else{
        cout << "Set done!" << endl;
//        printf("Again Open success!\n");
        return true;
    }
}

bool SerialPort::debugData(float * data){
    float send_float[4] = {(float)data[0], (float)data[1],
                           (float)data[2], (float)data[3]};
    unsigned char tail[4] {0x00, 0x00, 0x80, 0x7f};

    int statu1 = write(fd, send_float, sizeof(float)*4);
    int statu2 = write(fd, tail, 4);

    if(statu1 == -1 || statu2 == -1)
        return false;
    else
        return true;
}

//bool SerialPort::closeSerial() {
//    close(fd);
//}