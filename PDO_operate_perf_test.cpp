#include <string>
#include <stdexcept>

#include <stdio.h>

#include "can_poc351vtc.h"

#include <iostream>
#include <thread>
using namespace std::chrono;

const uint32_t node_id = 1;
const float freq_hz = 0.1;
float time_gap = 50000 / 3; //microseconds
int vel_lower = 0,vel_upper = 3000;
char accelerate = 'b';//accelerate(a) or decelerate(d) or both(b)
int acceleration = 1; int acceleration_limit = 10; //rps/s
char direction = 'b';//forward(f) or reverse(r) or both(b)
char fr = 'f';
canopen::Can can(500000, false);
system_clock::time_point sending_start, sending_end, receiving, sending_mid;
system_clock::duration gap_compensate;
const system_clock::duration cloud_time = seconds(6);
timespec send_start, send_end, mid;
const std::string file_name = "PDO_OP_perf_test";
FILE *fp;
void accel_determine(const int& accel);
static uint8_t count = 0;

void store_to_file()
{
    if (receiving > sending_start)
    {
        timespec send_start, send_end, recv;
        {
            auto secs = time_point_cast<seconds>(sending_start);
            auto ns = time_point_cast<nanoseconds>(sending_start) - time_point_cast<nanoseconds>(secs);
            send_start = {.tv_sec = secs.time_since_epoch().count(), .tv_nsec = ns.count()};
        }
        {
            auto secs = time_point_cast<seconds>(sending_end);
            auto ns = time_point_cast<nanoseconds>(sending_end) - time_point_cast<nanoseconds>(secs);
            send_end = {.tv_sec = secs.time_since_epoch().count(), .tv_nsec = ns.count()};
        }
        {
            auto secs = time_point_cast<seconds>(receiving);
            auto ns = time_point_cast<nanoseconds>(receiving) - time_point_cast<nanoseconds>(secs);
            recv = {.tv_sec = secs.time_since_epoch().count(), .tv_nsec = ns.count()};
        }

        fprintf(fp, "%ld\t%ld\t%ld\t%ld\t%ld\t%ld\n",
            send_start.tv_sec, send_start.tv_nsec,
            send_end.tv_sec, send_end.tv_nsec,
            recv.tv_sec, recv.tv_nsec);
    }
    else
    {
        timespec send;
        {
            auto secs = time_point_cast<seconds>(sending_start);
            auto ns = time_point_cast<nanoseconds>(sending_start) - time_point_cast<nanoseconds>(secs);
            send = {.tv_sec = secs.time_since_epoch().count(), .tv_nsec = ns.count()};
        }

        fprintf(fp, "%ld\t%ld\t0\t0\t0\t0\n",
            send.tv_sec, send.tv_nsec);
    }
}

void catch_sending_start_time()
{
    sending_start = system_clock::now();
}

void catch_sending_end_time()
{
    sending_end = system_clock::now();
}

void catch_receiving_time()
{
    receiving = system_clock::now();
}

void catch_sending_mid_time()
{
    sending_mid = system_clock::now();
    {
        auto secs = time_point_cast<seconds>(sending_mid);
        auto ns = time_point_cast<nanoseconds>(sending_mid) - time_point_cast<nanoseconds>(secs);
        mid = {.tv_sec = secs.time_since_epoch().count(), .tv_nsec = ns.count()};
    }
}

void msg_echo(const canopen::Can::Msg *msg)
{
    if (msg->identifier != (0x180 | node_id) && msg->identifier != (0x200 | node_id))
    {
        printf("CAN(PDO) message id compare fail\n");
        printf("%x\t",msg->identifier);
        for(int a = 0; a <= 7; a++)
        {
            printf("%x\t",msg->data[a]);
        }
        printf("\n");
    }
        
    else if (msg->data.size() != 8)
        {
            printf("Receive unknown message\n");
            for(int a = 0; a <= 7; a++)
            {
                printf("%x\t",msg->data[a]);
            }
        }

    else
    {
        catch_receiving_time();
        double a = msg->data[4];
        double b = msg->data[5]<<8;
        double c = msg->data[6]<<16;
        double d = msg->data[7]<<24;
        double i = a+b+c+d;
        i *= 3.6621/1e4;
        //printf("%F\n",i);
        for(int a = 0; a <= 7; a++)
        {
            fprintf(fp,"%x\t",msg->data[a]);
        }
        fprintf(fp,"%g\n",i);
    }
}

void setting_TPDO()
{
    usleep(100000);
    {   // Disable TPDO function
        std::vector<uint8_t> data = {
            0x23, (node_id - 1), 0x18, 0x01,
            (0x80 | node_id), 0x01, 0x00, 0x80
        };
        if (!can.send(0x600 | node_id, &data))
            throw std::logic_error("Disable TPDO function failed\n");
    }
    usleep(100000);
    {   // Clear TPDO mapping objects
        std::vector<uint8_t> data = {
            0x23, (node_id - 1), 0x1A, 0x00,
            0x00, 0x00, 0x00, 0x00
        };
        if (!can.send(0x600 | node_id, &data))
            throw std::logic_error("Clear TPDO mapping objects failed\n");
    }
    usleep(100000);
    {   // Set TPDO object queue
        {   // Actual Position
            std::vector<uint8_t> data = {
                0x23, (node_id - 1), 0x1A, 0x01,
                0x20, 0x00, 0x63, 0x60
            };
            if (!can.send(0x600 | node_id, &data))
                throw std::logic_error("Set TPDO object queue.1 failed\n");
        }
        usleep(100000);
        {   // Actual Speed
            std::vector<uint8_t> data = {
                0x23, (node_id - 1), 0x1A, 0x02,
                0x20, 0x00, 0x6C, 0x60
            };
            if (!can.send(0x600 | node_id, &data))
                throw std::logic_error("Set TPDO object queue.2 failed\n");
        }
    }
    usleep(100000);
    {   // Set TPDO mapping object number
        std::vector<uint8_t> data = {
            0x23, (node_id - 1), 0x1A, 0x00,
            0x02, 0x00, 0x00, 0x00
        };
        if (!can.send(0x600 | node_id, &data))
            throw std::logic_error("Set TPDO mapping object number failed\n");
    }
    usleep(100000);
    {   // Set TPDO configuration
        {   // Transmission type
            std::vector<uint8_t> data = {
                0x23, (node_id - 1), 0x18, 0x02,
                0x01, 0x00, 0x00, 0x00
            };
            if (!can.send(0x600 | node_id, &data))
                throw std::logic_error("Set TPDO transmission type failed\n");
        }
        usleep(100000);
        {   // Inhibit time
            std::vector<uint8_t> data = {
                0x23, (node_id - 1), 0x18, 0x03,
                0x00, 0x00, 0x00, 0x00
            };
            if (!can.send(0x600 | node_id, &data))
                throw std::logic_error("Set TPDO inhibit time failed\n");
        }
        usleep(100000);
        {   // Event timer
            std::vector<uint8_t> data = {
                0x23, (node_id - 1), 0x18, 0x05,
                0x00, 0x00, 0x00, 0x00
            };
            if (!can.send(0x600 | node_id, &data))
                throw std::logic_error("Set TPDO event timer failed\n");
        }
    }
    usleep(100000);
    {   // Enable TPDO function
        std::vector<uint8_t> data = {
            0x23, (node_id - 1), 0x18, 0x01,
            (0x80 | node_id), 0x01, 0x00, 0x00
        };
        if (!can.send(0x600 | node_id, &data))
            throw std::logic_error("Enable TPDO function failed\n");
    }
    usleep(100000);
}
void setting_RPDO()
{
    usleep(100000);
    {   // Disable RPDO function
        std::vector<uint8_t> data = {
            0x23, (node_id - 1), 0x14, 0x01,
            (0x80 | node_id), 0x01, 0x00, 0x80
        };
        if (!can.send(0x600 | node_id, &data))
            throw std::logic_error("Disable RPDO function failed\n");
    }
    usleep(100000);
    {   // Clear TPDO mapping objects
        std::vector<uint8_t> data = {
            0x23, (node_id - 1), 0x16, 0x00,
            0x00, 0x00, 0x00, 0x00
        };
        if (!can.send(0x600 | node_id, &data))
            throw std::logic_error("Clear RPDO mapping objects failed\n");
    }
    usleep(100000);
    {   // Set TPDO object queue
        {   // Actual Position
            std::vector<uint8_t> data = {
                0x23, (node_id - 1), 0x16, 0x01,
                0x20, 0x00, 0xff, 0x60
            };
            if (!can.send(0x600 | node_id, &data))
                throw std::logic_error("Set RPDO object queue.1 failed\n");
        }
    }
    usleep(100000);
    {   // Set TPDO mapping object number
        std::vector<uint8_t> data = {
            0x23, (node_id - 1), 0x16, 0x00,
            0x01, 0x00, 0x00, 0x00
        };
        if (!can.send(0x600 | node_id, &data))
            throw std::logic_error("Set RPDO mapping object number failed\n");
    }
    usleep(100000);
    {   // Set TPDO configuration
        {   // Transmission type
            std::vector<uint8_t> data = {
                0x23, (node_id - 1), 0x14, 0x02,
                0xfe, 0x00, 0x00, 0x00
            };
            if (!can.send(0x600 | node_id, &data))
                throw std::logic_error("Set RPDO transmission type failed\n");
        }
        usleep(100000);
        {   // Inhibit time
            std::vector<uint8_t> data = {
                0x23, (node_id - 1), 0x14, 0x03,
                0x00, 0x00, 0x00, 0x00
            };
            if (!can.send(0x600 | node_id, &data))
                throw std::logic_error("Set RPDO inhibit time failed\n");
        }
        usleep(100000);
        {   // Event timer
            std::vector<uint8_t> data = {
                0x23, (node_id - 1), 0x14, 0x05,
                0x00, 0x00, 0x00, 0x00
            };
            if (!can.send(0x600 | node_id, &data))
                throw std::logic_error("Set RPDO event timer failed\n");
        }
    }
    usleep(100000);
    {   // Enable TPDO function
        std::vector<uint8_t> data = {
            0x23, (node_id - 1), 0x14, 0x01,
            (0x00 | node_id), 0x02, 0x00, 0x00
        };
        if (!can.send(0x600 | node_id, &data))
            throw std::logic_error("Enable RPDO function failed\n");
    }
    usleep(100000);
}

void setup_speed_mode()
{
    usleep(100000);
    {   // Set operation mode
        std::vector<uint8_t> data = {
            0x2F, 0x60, 0x60, 0x00,
            3, 0x00, 0x00, 0x80
        };
        if (!can.send(0x600 | node_id, &data))
            throw std::logic_error("Set operation mode failed\n");
    }
    usleep(100000);
    {   // Set target speed = 0 rpm (dec = rpm*512*encoder/1875 = rpm*512*10000/1875)
        std::vector<uint8_t> data = {
            0x23, 0xFF, 0x60, 0x00,
            0x00, 0x00, 0x00, 0x00
        };
        if (!can.send(0x600 | node_id, &data))
            throw std::logic_error("Set target speed = 1000 failed\n");
    }
    usleep(100000);
    accel_determine(acceleration_limit);
    usleep(100000);
    {   // Set controlword = 0x0080 (clear error)
        std::vector<uint8_t> data = {
            0x2B, 0x40, 0x60, 0x00,
            0x80, 0x00, 0x00, 0x00
        };
        if (!can.send(0x600 | node_id, &data))
            throw std::logic_error("Set controlword = 0x0080 (clear error) failed\n");
    }
    usleep(100000);
    {   // Set controlword = 0x0006 (disable)
        std::vector<uint8_t> data = {
            0x2B, 0x40, 0x60, 0x00,
            0x06, 0x00, 0x00, 0x00
        };
        if (!can.send(0x600 | node_id, &data))
            throw std::logic_error("Set controlword = 0x0006 (disable) failed\n");
    }
    usleep(100000);
    {   // Set controlword = 0x0007 (switch on)
        std::vector<uint8_t> data = {
            0x2B, 0x40, 0x60, 0x00,
            0x07, 0x00, 0x00, 0x00
        };
        if (!can.send(0x600 | node_id, &data))
            throw std::logic_error("Set controlword = 0x0007 (switch on) failed\n");
    }
    usleep(100000);
    {   // Set controlword = 0x000f (operaiton)
        std::vector<uint8_t> data = {
            0x2B, 0x40, 0x60, 0x00,
            0x0F, 0x00, 0x00, 0x00
        };
        if (!can.send(0x600 | node_id, &data))
            throw std::logic_error("Set controlword = 0x000f (operaiton) failed\n");
    }

    usleep(100000);
}

void close_operation()
{
    usleep(100000);
    {   // Set controlword = 0x0007 (switch on)
        std::vector<uint8_t> data = {
            0x2B, 0x40, 0x60, 0x00,
            0x07, 0x00, 0x00, 0x00
        };
        if (!can.send(0x600 | node_id, &data))
            throw std::logic_error("Set controlword = 0x0007 (switch on) failed\n");
    }
    usleep(100000);
    {   // Set controlword = 0x0006 (disable)
        std::vector<uint8_t> data = {
            0x2B, 0x40, 0x60, 0x00,
            0x06, 0x00, 0x00, 0x00
        };
        if (!can.send(0x600 | node_id, &data))
            throw std::logic_error("Set controlword = 0x0006 (disable) failed\n");
    }
    usleep(100000);
    {   // NMT enable all devices
        std::vector<uint8_t> data = {
            0x01, 0x00
        };
        if (!can.send(0x00, &data))
            throw std::logic_error("NMT enable all devices failed\n");
    }
    usleep(100000);
}
void velocity_determine(const int& vel)
{
    uint8_t data_a, data_b, data_c, data_d;
    int DEC;
    DEC = data_a = data_b = data_c = data_d = 0;
    DEC = vel * 1e4 / 3.6621; //3.6621 = 1875 / 512; encoder resolution = 10000
    data_a = DEC & 255;
    data_b = DEC >> 8 & 255;
    data_c = DEC >> 16 & 255;
    data_d = DEC >> 24 & 255;
    std::vector<uint8_t> data = {
        data_a, data_b, data_c, data_d
    };
    if(!can.send(0x200 | node_id, &data))
    printf("RPDO fail\n");
}
void accel_determine(const int& accel)
{
    uint8_t data_a, data_b, data_c, data_d;
    int DEC;
    DEC = data_a = data_b = data_c = data_d = 0;
    DEC = accel * 1e4 / 61; //61 = 4000000 / 65536; encoder resolution = 10000
    data_a = DEC & 255;
    data_b = DEC >>  8 & 255;
    data_c = DEC >> 16 & 255;
    data_d = DEC >> 24 & 255;
    std::vector<uint8_t> data = {
        0x23, 0x83, 0x60, 0x00,
        data_a, data_b, data_c, data_d
    };
    can.send(0x600 | node_id, &data);
    usleep(time_gap);
    data = {
        0x23, 0x84, 0x60, 0x00,
        data_a, data_b, data_c, data_d
    };
    can.send(0x600 | node_id, &data);
}
void scenario()
{
    switch (accelerate)
    {
        case 'a' :
        case 'b' :
        {   //the rising part
            int vel = vel_lower;
            if (fr == 'f')
            {
                while(vel < (vel_upper+1))
                {
                    catch_sending_start_time();
                    velocity_determine(vel);
                    //std::cout<<vel<<"\n";
                    fprintf(fp,"target speed : \t%d\n",vel);
                    catch_sending_mid_time();
                    gap_compensate = sending_mid - sending_start;
                    usleep ((uint32_t) time_gap);
                    std::vector<uint8_t> data = {
                        count++
                    };
                    can.send(0x80, &data);
                    catch_sending_end_time();
                    gap_compensate = sending_end - sending_mid;
                    usleep((uint32_t) (time_gap));
                    vel += (acceleration*60*time_gap/1e6*2+1);
                    store_to_file();
                }
            }
            else
            {
                while(vel > (vel_upper-1))
                {
                    catch_sending_start_time();
                    velocity_determine(vel);
                    //std::cout<<vel<<"\n";
                    fprintf(fp,"target speed : \t%d\n",vel);
                    catch_sending_mid_time();
                    gap_compensate = sending_mid - sending_start;
                    usleep ((uint32_t) (time_gap));
                    std::vector<uint8_t> data = {
                        count++
                    };
                    can.send(0x80, &data);
                    catch_sending_end_time();
                    gap_compensate = sending_end - sending_mid;
                    usleep((uint32_t) (time_gap));
                    vel -= (acceleration*60*time_gap/1e6*2+1);
                    store_to_file();
                }
            }
            if (accelerate == 'a')
            {
                break;
            }
        }
        case 'd' :
        {   //the section is from rising to flatten part
            system_clock::time_point ascend_to_flat = system_clock::now();
            system_clock::time_point now = system_clock::now();
            system_clock::duration pass_time = now - ascend_to_flat;
            while (cloud_time > pass_time)
            {
                velocity_determine(vel_upper);
                fprintf(fp,"target speed : \t%d\n",vel_upper);
                usleep ((uint32_t) (time_gap));
                std::vector<uint8_t> data = {
                        count++
                    };
                can.send(0x80, &data);
                usleep ((uint32_t) (time_gap));
                now = system_clock::now();
                pass_time = now - ascend_to_flat;
                store_to_file();
            }
            //the descend part
            std::cout << "dece" << std::endl;
            int vel = vel_upper;
            if (fr == 'f')
            {
                while(vel > (vel_lower-1))
                {
                    catch_sending_start_time();
                    velocity_determine(vel);
                    //std::cout<<vel<<"\n";
                    fprintf(fp,"target speed : \t%d\n",vel);
                    catch_sending_mid_time();
                    gap_compensate = sending_mid - sending_start;
                    usleep ((uint32_t) (time_gap) );
                    std::vector<uint8_t> data = {
                        count++
                    };
                    can.send(0x80, &data);
                    catch_sending_end_time();
                    gap_compensate = sending_end - sending_mid;
                    usleep((uint32_t) (time_gap ) );
                    vel -= (acceleration*60*time_gap/1e6*2);
                    store_to_file();
                }
            }
            else
            {
                while(vel < (vel_lower-1))
                {
                    catch_sending_start_time();
                    velocity_determine(vel);
                    //std::cout<<vel<<"\n";
                    fprintf(fp,"target speed : \t%d\n",vel);
                    catch_sending_mid_time();
                    gap_compensate = sending_mid - sending_start;
                    usleep ((uint32_t) (time_gap) );
                    std::vector<uint8_t> data = {
                        count++
                    };
                    can.send(0x80, &data);
                    catch_sending_end_time();
                    gap_compensate = sending_end - sending_mid;
                    usleep((uint32_t) (time_gap ) );
                    vel += (acceleration*60*time_gap/1e6*2);
                    store_to_file();
                }
            }
            break;
        }
    }
}
void NMT_enable()
{
        std::vector<uint8_t> data = 
        {
            1,1
        };
        if (!can.send(0x000, &data))
        {
            printf("000fail\n");
        }
        usleep(50000);
}
std::string d1 = "There should be 4 inputs after Kinco_PDO_OP.\n";
std::string d2 = "The sequence is \n.the maximum acceleration\n.accelerate(a) or decelerate(b) or both(b)\n";
std::string d3 = ".forward(f) or reverse(r) or both(b)\n.the initial acceleration\n";
std::string d4 = "for example : sudo ./kinco_PDO_OP 10 b b 5\n";
std::string description = d1 + d2 + d3 + d4;
char option ;
int language;
std::string c1 = "PDO_OP之後應該有4個輸入。\n順序是\n最大加速度\n加速(a)減速(d)或兩者(b)\n正轉(f)反轉(r)或兩者(b)\n初始加速度";

int main(int argc, char *argv[])
{
    if(argc == 1)
    {
        std::cout << "English please press 0\t中文請按1\n";
        std::cin >> language;
        if(language == 0)
        {
           std::cout<<description<<"\n"<<"If you want to set it yourself, please press 0 to exit.\n"
            <<"If you want the default setting ,please press other keys.\n";
            std::cin >> option;
            if (option == '0')
            {
                return 0;
            }
            else
            {
                std::cout << "The test shall begin.\n";
            } 
        }
        else
        {
            std::cout<<c1<<"\n"<<"如果要自行設置，請按0退出。.\n"
            <<"如果要使用默認設置，請按其他。\n";
            std::cin >> option;
            if (option == '0')
            {
                return 0;
            }
            else
            {
                std::cout << "測試開始。\n";
            } 
        }
        
    }
    else if(argc == 5)
    {
        acceleration_limit = atoi(argv[1]);
        accelerate = *argv[2];
        direction = *argv[3];
        acceleration = atoi(argv[4]);
    }
    else
    {
        std::cout<<"incorrect input\n"<<description<<"\n";
        return 0;
    }
    
    if (!can.start())
    {
        printf("client CAN start fail\n");
        return 0;
    }

    setting_TPDO();
    setting_RPDO();

    printf("client frequency = %f Hz\n", 1.0/freq_hz);
    while(acceleration <= acceleration_limit)
    {
        boost::signals2::scoped_connection connection = can.registerRecvCb(msg_echo);
        setup_speed_mode();
        NMT_enable();
        std::string str = file_name + 
        "acceleration_ " +
        std::to_string(acceleration)+
        ".xlsx";
        fp = fopen(str.c_str(), "w");
        if (fp == NULL)
        {
            printf("file open fail, program closing\n");
            return 0;
        }
        else
        {
            fprintf(fp, "Kinco performance test using PDO\n");
            fprintf(fp, "sending start\tsending end\treceiving\n");
            fprintf(fp, "sec\tnsec\tsec\tnsec\tsec\tnsec\treal_vel\n");
        }
        std::cout << "acceleration " << acceleration << std::endl;
        switch (direction)
        {
        case 'f':
        case 'b':
            fr = 'f';
            std::cout << "foward"<<std::endl;
            scenario();
            if(direction == 'f')
            {
                break;
            }
        case 'r':
            fr = 'r';
            std::cout << "reverse"<<std::endl;
            vel_lower *= -1; vel_upper *= -1;
            scenario();
            vel_lower *= -1; vel_upper *= -1;
            break;
        }
        acceleration ++;
        //accel_determine(acceleration);
        fclose(fp);
        connection.disconnect();
        close_operation();
    }
    return 0;
}
