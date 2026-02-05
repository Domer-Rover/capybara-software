// Copyright (c) 2023 Eric Cox
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

#include <cstdint>
#include <iostream>
#include <memory>
#include <string>

namespace roboclaw_serial
{

class SerialDevice
{
public:
  typedef std::shared_ptr<SerialDevice> SharedPtr;

  SerialDevice() = default;

  explicit SerialDevice(const std::string device) {connect(device);}
  virtual ~SerialDevice() {disconnect();}

  virtual bool connect(const std::string & device)
  {
    fd_ = open(device.c_str(), O_RDWR | O_NOCTTY);
    connected_ = fd_ != -1;

    if (connected_) {
      setSerialDeviceOptions();
    } else {
      std::cerr << "Failed to open serial device: " << device << std::endl;
      perror("Error");
    }

    return connected_;
  }

  virtual void disconnect()
  {
    if (connected_) {
      close(fd_);
      connected_ = false;
    }
  }

  bool connected() const {return connected_;}

  virtual std::size_t write(const std::byte * buffer, std::size_t count)
  {
    ssize_t result = ::write(fd_, buffer, count);
    if (result < 0) {
      // Error writing to device
      throw std::range_error("Error writing to the device!");
    }
    return static_cast<std::size_t>(result) == count;
  }

  virtual std::size_t read(std::byte * buffer, std::size_t count)
  {
    std::size_t total_read = 0;
    
    while (total_read < count) {
      fd_set set;
      struct timeval timeout;

      /* Initialize the file descriptor set. */
      FD_ZERO(&set);
      FD_SET(fd_, &set);

      /* Initialize the timeout data structure. */
      timeout.tv_sec = 0;
      timeout.tv_usec = 100000;  // 100ms timeout per chunk

      /* select returns 0 if timeout, 1 if input available, -1 if error. */
      int res = select(FD_SETSIZE, &set, NULL, NULL, &timeout);
      if (res < 0) {
        throw std::range_error("Error reading from the serial device!");
      } else if (res == 0) {
        throw std::runtime_error("Read timeout!");
      }
      
      ssize_t result = ::read(fd_, buffer + total_read, count - total_read);
      if (result < 0) {
        throw std::range_error("Error reading from the serial device!");
      } else if (result == 0) {
        throw std::runtime_error("Read timeout!");
      }
      
      total_read += static_cast<std::size_t>(result);
    }

    return total_read;
  }

protected:
  bool connected_ = false;

private:
  void setSerialDeviceOptions()
  {
    struct termios options;
    memset(&options, 0, sizeof(options));
    
    // Set baud rate and flags together (B38400 is part of c_cflag)
    options.c_cflag = B38400 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    
    // Set VMIN and VTIME for read behavior
    options.c_cc[VMIN] = 0;   // Non-blocking read
    options.c_cc[VTIME] = 1;  // 100ms timeout (in tenths of seconds)
    
    tcflush(fd_, TCIFLUSH);
    tcsetattr(fd_, TCSANOW, &options);
  }

  int fd_ = -1;
};

}  // namespace roboclaw_serial
