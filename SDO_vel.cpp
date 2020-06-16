#include <string>

#include <stdio.h>

#include <chrono>

#include <iostream>

using namespace std::chrono;

const uint32_t node_id = 1;
const float freq_hz = 0.1;
const int running_hours = 4;
const system_clock::duration running_time = seconds(16);
const system_clock::duration recording_time = seconds(30);
bool time_up = true;
int vel_lower = -1000,vel_upper = 2000;
bool accelerate = true;

canopen::Can can(500000, false);
system_clock::time_point sending_start, sending_end, receiving;

system_clock::time_point while_start;

const std::string file_name = "SDO_vel_cal";
FILE *fp;

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

        fprintf(fp, "%ld\t%ld\t%ld\t%ld\t%ld\t%ld\t",
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

        fprintf(fp, "%ld\t%ld\t0\t0\t0\t0\t",
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

void msg_echo(const canopen::Can::Msg *msg)
{
    if (msg->identifier != (0x580 | node_id))
        printf("CAN message id compare fail\n");
    else if (msg->data.size() != 8)
        printf("Receive non-SDO message\n");
    else if (msg->data[0] == 0x80)
        printf("Receive device SDO abort response\n");
    else if ((msg->identifier == (0x580 | node_id)) && (msg->data[0] == 0x43||(msg->data[0] == 0x60 
    && msg->data[1] == 0xff)))
    {    
    //if(time_up == false)
    //fprintf(fp,"\n0\t0\t0\t0\t0\t0\t");
    double a = msg->data[4];
    double b = msg->data[5]<<8;
    double c = msg->data[6]<<16;
    double d = msg->data[7]<<24;
    double i = a+b+c+d;
    i = i *3.6621/1e4;
    for(int a = 0; a < 4; a++)
    {
        fprintf(fp,"%x\t",msg->data[a]);
    }
    fprintf(fp,"%g\n",i);
    }
    catch_sending_end_time();
    store_to_file();
}
void velocity_determine(const int& vel)
{
    uint8_t data_a, data_b, data_c, data_d;
    int encoder;
    encoder = data_a = data_b = data_c = data_d = 0;
    encoder = vel*1e4/3.6621;
    data_a = encoder & 255;
    data_b = encoder >> 8 & 255;
    data_c = encoder >> 16 & 255;
    data_d = encoder >> 24 & 255;                    
    std::vector<uint8_t> data = {
        0x23, 0xFF, 0x60, 0x00,
        data_a, data_b, data_c, data_d
    };
    can.send(0x600 | node_id, &data);
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
    
    {   // Set target speed = 1000 rpm (dec = rpm*512*encoder/1875 = rpm*512*10000/1875)
        if (accelerate == true)
        {
            velocity_determine(vel_lower);
        }
        else
        {
            velocity_determine(vel_upper);
        }
    }
    
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
}

void terminal_speed()
{
    // Set target speed = 0 rpm (dec = rpm*512*encoder/1875 = rpm*512*10000/1875)
    velocity_determine(0);
}

int main()
{
    can.registerRecvCb(msg_echo);

    if (!can.start())
    {
        printf("client CAN start fail\n");
        return 0;
    }

    std::string str = file_name + 
        //std::to_string(running_hours) + "hrs_" +
        //std::to_string(1.0/freq_hz) + "Hz" +
        ".xlsx";
    fp = fopen(str.c_str(), "w");
    if (fp == NULL)
    {
        printf("file open fail, program closing\n");
        return 0;
    }
    else
    {
        fprintf(fp, "SDO request/response performance test\n");
        fprintf(fp, "recording time:%i hrs\tfrequency:%f Hz\n", running_hours, 1.0/freq_hz);
        //fprintf(fp, "=============== value types ===============\n");
        fprintf(fp, "sending start\tsending end\treceiving\n");
        fprintf(fp, "sec\tnsec\tsec\tnsec\tsec\tnsec\treal_vel\n");
        //fprintf(fp, "===============  recording  ===============\n");
    }
    setup_speed_mode();

    printf("client frequency = %f Hz\n", 1.0/freq_hz);
    while_start = system_clock::now();
    if(accelerate = true)
    {   
        catch_sending_start_time();
        int vel = vel_lower;
        for(int v = vel_lower;v < (vel_upper+1);v++)
        {
            velocity_determine(vel);
            std::cout<<vel<<"\n";
            usleep((uint32_t)(50000/6));
            std::vector<uint8_t> data = {
                0x40, 0x6c, 0x60, 0x00,
                0x00, 0x00, 0x00, 0x00
            };
            can.send(0x600 | node_id, &data);
            usleep((uint32_t)(50000/6));
            vel = vel + 1;
        }
    }
    else
    {
        catch_sending_start_time();
        int vel = vel_upper;
        for(int v = vel_upper;v > (vel_lower-1);v--)
        {
            velocity_determine(vel);
            std::cout<<vel<<"\n";
            usleep((uint32_t)(50000/6));
            std::vector<uint8_t> data = {
                0x40, 0x6c, 0x60, 0x00,
                0x00, 0x00, 0x00, 0x00
            };
            can.send(0x600 | node_id, &data);
            usleep((uint32_t)(50000/6));
            vel = vel - 1;
        }
    }
    while (true)
    {
        canopen::Can::Status status = can.getStatus();
        if (status.bus_off)
        {
            printf("CAN bus shotdown, client program closing\n");
            break;
        }
        /*
        std::vector<uint8_t> data = {
            0x2F, 0x60, 0x60, 0x00,
            0x03, 0x00, 0x00, 0x00
        };
        catch_sending_start_time();
        if (!can.send(0x600 | node_id, &data))
        {
            printf("CAN send fail, client program closing\n");
            break;
        }
        data = {
            0x2B, 0x40, 0x60, 0x00,
            0x0F, 0x00, 0x00, 0x00
        };
        if (!can.send(0x600 | node_id, &data))
        {
            printf("CAN send fail, client program closing\n");
            break;
        }
        data = {
            0x23, 0xff, 0x60, 0x00,
            0x14, 0x00, 0x7d, 0x00
        };
        if (!can.send(0x600 | node_id, &data))
        {
            printf("CAN send fail, client program closing\n");
            break;
        }
        catch_sending_end_time();
        */        
        std::vector<uint8_t> data = {
            0x40, 0x6c, 0x60, 0x00,
            0x00, 0x00, 0x00, 0x00
        };
        if (!can.send(0x600 | node_id, &data))
        {
            printf("CAN send fail, first_part, client program closing\n");
            break;
        }
        
        usleep((uint32_t)(freq_hz * 5000.0));
        store_to_file();
        system_clock::time_point now = system_clock::now();
        system_clock::duration pass_time = now - while_start;        
        if (running_time <= pass_time)
        {
            //printf("time out, program closing\n");
            time_up = false;
            terminal_speed();              
            if( pass_time < recording_time )
            {
                data = {
                0x40, 0x6c, 0x60, 0x00,
                0x00, 0x00, 0x00, 0x00
                };
                can.send(0x600 | node_id, &data);
                usleep((uint32_t)(freq_hz * 5000.0));
                data = {
                0x40, 0x63, 0x60, 0x00,
                0x00, 0x00, 0x00, 0x00
                };
                can.send(0x600 | node_id, &data);
                usleep((uint32_t)(freq_hz * 5000.0));
                fprintf(fp,"%f\n",(float)pass_time.count());
            }
            else
            {
                fclose(fp);
                break;
            }
        }
        else
        {
            static uint8_t count;
            float percentage = (float)pass_time.count() / (float)running_time.count();
            printf("%02x req/resp percent=%.1f %%\n", count++, percentage * 100);
        }
    }
    return 0;
}