#include "ibus.h"
#include "serial/serial.h"

#include <stdint.h>
#include <thread>
#include <string>
#include <iostream>

ibusdriver::ibusdriver(std::string &serial_name, uint32_t serial_baudrate)    
            : serial_port_(new serial::Serial(serial_name, serial_baudrate, serial::Timeout::simpleTimeout(100)))
            , thread_running_(true)
            , th_(nullptr)
  {
    
    
    // init ibus state
    ibus_init(&ibus_data_state, channel_count);

    // open serial
    if (serial_port_->isOpen())
    {
      std::cout << "ibus Serial is opened." << std::endl;
      th_ = new std::thread(&ibusdriver::ThreadCallBack, this);
    }
    else
    {
      std::cout<< "[ERROR] ibus Serial is not opened." <<std::endl;
    }    
  }

ibusdriver::~ibusdriver()
{        
  thread_running_ = false;
  if (nullptr != th_)
  {
      th_->join();
  }
  serial_port_->close();

  delete serial_port_;
  std::cout << "Serial END" << std::endl;   
}

void ibusdriver::ibus_init(struct ibus_state* state, uint_fast8_t channel_count) {
  state->state = 0;
  state->channel_count = channel_count;
}

int ibusdriver::ibus_read(struct ibus_state* state, uint16_t* data, uint8_t ch) {
  switch (state->state) {
    
    case 0:
      if (ch == 0x20) 
      {
        state->checksum = 0xFFFF - 0x20;
        state->state = 1;
      }
      break;

    case 1:
      if (ch == 0x40) 
      {
        state->state = 2;
        state->checksum -= ch;
      } 
      else 
      {
        // Unknown packet type
        state->state = 0;
      }
      break;

    case 30:
      state->datal = ch;
      state->state = 31;
      break;

    case 31: 
      {
        uint_fast16_t checksum = (ch << 8) | state->datal;
        state->state = 0;
        if (checksum == state->checksum)  return 0;
      } 
      break;
    default:
      // Ignore these bytes if we've filled all of the channels
      if (state->state / 2 <= state->channel_count) 
      {
        if ((state->state & 1) == 0) 
        {
          // Data low byte
          state->datal = ch;
        }
        else
        {
          // Data high byte
          data[(state->state / 2) - 1] = (ch << 8) | state->datal;
        }
      }
      state->checksum -= ch;
      ++state->state;
      break;
    }
    return -1;
}

void ibusdriver::ThreadCallBack(void)
{   
  uint8_t rcv_char;
  uint16_t data_out[channel_count];
  while (thread_running_ == true && serial_port_->read(&rcv_char, 1) == 1)
  {
    if(ibus_read(&ibus_data_state, data_out, rcv_char) == 0) 
    {
      mut_.lock();
      memcpy(final_output,data_out,channel_count*sizeof(uint16_t));
      // for (int i = 0; i < channel_count; i++) printf("%d ", data_out[i]);
      // printf("\n");
      mut_.unlock();
    }    
  }
}