/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once
#ifdef PROTO_FOUND
#include <string>
#include <iostream>
#include <memory.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <vector>
#include <cstdlib>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#define SPLIT_SIZE 5000
#define MAX_RECEIVE_LENGTH 5200

namespace robosense
{
namespace lidar
{
enum class DataEndianType
{
  RS_BIG_ENDIAN = 0,
  RS_SMALL_ENDIAN = 1
};
template <typename T>
class CRSEndian
{
public:
  typedef typename std::shared_ptr<CRSEndian> Ptr;
  typedef typename std::shared_ptr<const CRSEndian> ConstPtr;

public:
  CRSEndian()
  {
    unsigned char* p_char = (unsigned char*)(&magic_data_);
    if (*p_char == magic_big_endian_[0] && (*(p_char + 1)) == magic_big_endian_[1] &&
        (*(p_char + 2)) == magic_big_endian_[2] && (*(p_char + 3)) == magic_big_endian_[3])
    {
      host_endian_type_ = DataEndianType::RS_BIG_ENDIAN;
    }
    else
    {
      host_endian_type_ = DataEndianType::RS_SMALL_ENDIAN;
    }
  }

public:
  DataEndianType getHostEndian()
  {
    return host_endian_type_;
  }

  int toTargetEndianArray(T t, void* p_array, const unsigned int& max_size, const DataEndianType& dst_endian)
  {
    unsigned int t_size = sizeof(T);
    assert(t_size <= max_size);
    assert(p_array != nullptr);
    if (host_endian_type_ != dst_endian)
    {
      char* p_data_ = (char*)(&t);
      unsigned int t_size_h = t_size / 2;
      for (unsigned int idx = 0; idx < t_size_h; ++idx)
      {
        char tmp = p_data_[idx];
        unsigned int swapIdx = t_size - idx - 1;
        p_data_[idx] = p_data_[swapIdx];
        p_data_[swapIdx] = tmp;
      }
    }
    memcpy(p_array, &t, t_size);
    return 0;
  }

  int toHostEndianValue(T& t, const void* p_array, const unsigned int& max_size, const DataEndianType& src_endian)
  {
    unsigned int t_size = sizeof(T);
    assert(p_array != nullptr);
    assert(t_size <= max_size);
    if (src_endian != host_endian_type_)
    {
      char* p_data_ = (char*)(p_array);
      unsigned int t_size_h = t_size / 2;
      for (unsigned idx = 0; idx < t_size_h; ++idx)
      {
        char tmp = p_data_[idx];
        unsigned int swapIdx = t_size - idx - 1;
        p_data_[idx] = p_data_[swapIdx];
        p_data_[swapIdx] = tmp;
      }
    }
    memcpy(&t, p_array, t_size);
    return 0;
  }

private:
  DataEndianType host_endian_type_;
  const unsigned int magic_data_ = 0xA1B2C3D4;
  const unsigned char magic_small_endian_[4] = { (unsigned char)0xD4, (unsigned char)0xC3, (unsigned char)0xB2,
                                                 (unsigned char)0xA1 };
  const unsigned char magic_big_endian_[4] = { (unsigned char)0xA1, (unsigned char)0xB2, (unsigned char)0xC3,
                                               (unsigned char)0xD4 };
};

struct alignas(16) ProtoMsgHeader
{
  unsigned int frame_num;
  unsigned int total_msg_cnt;
  unsigned int msg_id;
  unsigned int msg_length;
  unsigned int total_msg_length;

  ProtoMsgHeader()
  {
    frame_num = 0;
    total_msg_cnt = 0;
    msg_id = 0;
    msg_length = 0;
    total_msg_length = 0;
  }
  bool toTargetEndianArray(char* p_array, const unsigned int maxArraySize,
                           const DataEndianType dst_endian = DataEndianType::RS_BIG_ENDIAN) const
  {
    if (p_array == nullptr || maxArraySize < sizeof(ProtoMsgHeader))
    {
      return false;
    }

    CRSEndian<unsigned int> uint32Endian;
    CRSEndian<std::time_t> uint64Endian;

    int offset = 0;
    uint32Endian.toTargetEndianArray(frame_num, p_array + offset, maxArraySize - offset, dst_endian);

    offset += sizeof(unsigned int);
    uint32Endian.toTargetEndianArray(total_msg_cnt, p_array + offset, maxArraySize - offset, dst_endian);

    offset += sizeof(unsigned int);
    uint32Endian.toTargetEndianArray(msg_id, p_array + offset, maxArraySize - offset, dst_endian);

    offset += sizeof(unsigned int);
    uint32Endian.toTargetEndianArray(msg_length, p_array + offset, maxArraySize - offset, dst_endian);

    offset += sizeof(unsigned int);
    uint32Endian.toTargetEndianArray(total_msg_length, p_array + offset, maxArraySize - offset, dst_endian);
    return true;
  }
  bool toHostEndianValue(const char* p_array, const unsigned int arraySize,
                         const DataEndianType src_endian = DataEndianType::RS_BIG_ENDIAN)
  {
    if (p_array == nullptr || arraySize < sizeof(ProtoMsgHeader))
    {
      return false;
    }
    CRSEndian<unsigned int> uint32Endian;
    CRSEndian<std::time_t> uint64Endian;

    int offset = 0;
    uint32Endian.toHostEndianValue(frame_num, p_array + offset, arraySize - offset, src_endian);

    offset += sizeof(unsigned int);
    uint32Endian.toHostEndianValue(total_msg_cnt, p_array + offset, arraySize - offset, src_endian);

    offset += sizeof(unsigned int);
    uint32Endian.toHostEndianValue(msg_id, p_array + offset, arraySize - offset, src_endian);

    offset += sizeof(unsigned int);
    uint32Endian.toHostEndianValue(msg_length, p_array + offset, arraySize - offset, src_endian);

    offset += sizeof(unsigned int);
    uint32Endian.toHostEndianValue(total_msg_length, p_array + offset, arraySize - offset, src_endian);
    return true;
  }
};
using boost::asio::deadline_timer;
using boost::asio::ip::udp;
/**
 * @brief  Protobuf UDP transmission basic class
 * @note   Initialize socket sender and receiver, and define send function and receive function
 */
class ProtoCommunicator
{
public:
  ProtoCommunicator() = default;
  ~ProtoCommunicator() = default;

  /**
   * @brief  intilize the socket sender
   * @param  proto_send_port: destination port
   * @param  proto_send_ip: destination IP address
   * @retval 0: success -1: failed
   */
  inline int initSender(std::string proto_send_port, std::string proto_send_ip)
  {
    try
    {
      send_sock_ptr_.reset(new udp::socket(io_service_, udp::endpoint(udp::v4(), 0)));
      boost::asio::socket_base::broadcast option(true);
      send_sock_ptr_->set_option(option);
      udp::resolver resolver(io_service_);
      udp::resolver::query query(udp::v4(), proto_send_ip, proto_send_port);
      iterator_ = resolver.resolve(query);
    }
    catch (...)
    {
      RS_ERROR << "Proto Sender Port is already used! Abort!" << RS_REND;
      exit(-1);
    }
    return 0;
  }
  /**
   * @brief  Initialize the socket receiver
   * @param  proto_recv_port: the receiver port number
   * @retval 0:success -1:failed
   */
  inline int initReceiver(uint16_t proto_recv_port)
  {
    try
    {
      recv_sock_ptr_.reset(new udp::socket(io_service_, udp::endpoint(udp::v4(), proto_recv_port)));
      deadline_.reset(new deadline_timer(io_service_));
    }
    catch (...)
    {
      RS_ERROR << "Proto Receiver Port is already used! Abort!" << RS_REND;
      exit(-1);
    }
    deadline_->expires_at(boost::posix_time::pos_infin);
    checkDeadline();
    return 0;
  }

  /* message send function & message receive function */
  /**
   * @brief  the message send function
   * @note   serialize the message to protobuf and send it
   * @param  *pMsgData: data
   * @param  &header: header
   * @retval >=0 : success -1: failed
   */
  inline int sendProtoMsg(const void* pMsgData, const ProtoMsgHeader& header)
  {
    int sendLen = header.msg_length + sizeof(ProtoMsgHeader);
    char* sendBuffer = (char*)malloc(sendLen);
    memset(sendBuffer, 0, sendLen);
    header.toTargetEndianArray(sendBuffer, sizeof(ProtoMsgHeader), DataEndianType::RS_BIG_ENDIAN);
    if (header.msg_length > 0)
    {
      memcpy(sendBuffer + sizeof(ProtoMsgHeader), pMsgData, header.msg_length);
    }
    int ret = send_sock_ptr_->send_to(boost::asio::buffer(sendBuffer, sendLen), *iterator_);
    free(sendBuffer);
    return ret;
  }
  /**
   * @brief  message receive function
   * @note   receive the message and deserilize it
   * @param  *pMsgData: the output message
   * @param  msgMaxLen: max receive message length
   * @param  &header: output header
   * @retval >=0: success -1: failed
   */
  inline int receiveProtoMsg(void* pMsgData, const int msgMaxLen, ProtoMsgHeader& header)
  {
    deadline_->expires_from_now(boost::posix_time::seconds(1));
    boost::system::error_code ec = boost::asio::error::would_block;
    std::size_t ret = 0;
    char* pRecvBuffer = (char*)malloc(msgMaxLen + sizeof(ProtoMsgHeader));
    recv_sock_ptr_->async_receive(boost::asio::buffer(pRecvBuffer, msgMaxLen + sizeof(ProtoMsgHeader)),
                                  boost::bind(&ProtoCommunicator::handleReceive, _1, _2, &ec, &ret));
    do
    {
      io_service_.run_one();
    } while (ec == boost::asio::error::would_block);
    if (ec)
    {
      free(pRecvBuffer);
      return -1;
    }
    header.toHostEndianValue(pRecvBuffer, sizeof(ProtoMsgHeader), DataEndianType::RS_BIG_ENDIAN);
    if (ret < (std::size_t)(header.msg_length + sizeof(ProtoMsgHeader)))
    {
      free(pRecvBuffer);
      return -1;
    }
    if (header.msg_length > 0)
    {
      memcpy(pMsgData, pRecvBuffer + sizeof(ProtoMsgHeader), header.msg_length);
    }
    free(pRecvBuffer);
    return ret;
  }

  template <typename T>
  bool sendSplitMsg(const T& msg)
  {
    void* buf = malloc(msg.ByteSize() + SPLIT_SIZE);
    msg.SerializeToArray(buf, msg.ByteSize());
    int pkt_num = ceil(1.0 * msg.ByteSize() / SPLIT_SIZE);
    ProtoMsgHeader tmp_header;
    tmp_header.frame_num = msg.seq();
    tmp_header.msg_length = SPLIT_SIZE;
    tmp_header.total_msg_cnt = pkt_num;
    tmp_header.total_msg_length = msg.ByteSize();
    for (int i = 0; i < pkt_num; i++)
    {
      tmp_header.msg_id = i;
      void* tmp_buf = malloc(SPLIT_SIZE);
      memcpy(tmp_buf, (uint8_t*)buf + i * SPLIT_SIZE, SPLIT_SIZE);
      if (sendProtoMsg(tmp_buf, tmp_header) == -1)
      {
        free(tmp_buf);
        free(buf);
        return false;
      }
      free(tmp_buf);
      usleep(2);
    }
    free(buf);
    return true;
  }

  template <typename T>
  bool sendSingleMsg(const T& msg)
  {
    ProtoMsgHeader tmp_header;
    tmp_header.msg_length = msg.ByteSize();
    void* buf = malloc(tmp_header.msg_length);
    msg.SerializeToArray(buf, tmp_header.msg_length);
    if (sendProtoMsg(buf, tmp_header) == -1)
    {
      free(buf);
      return false;
    }
    free(buf);
    return true;
  }

private:
  inline void checkDeadline()
  {
    if (deadline_->expires_at() <= deadline_timer::traits_type::now())
    {
      recv_sock_ptr_->cancel();
      deadline_->expires_at(boost::posix_time::pos_infin);
    }
    deadline_->async_wait(boost::bind(&ProtoCommunicator::checkDeadline, this));
  }
  static void handleReceive(const boost::system::error_code& ec, std::size_t length, boost::system::error_code* out_ec,
                             std::size_t* out_length)
  {
    *out_ec = ec;
    *out_length = length;
  }

private:
  /* UDP socket related varibles */
  boost::asio::io_service io_service_;
  udp::resolver::iterator iterator_;
  std::unique_ptr<udp::socket> send_sock_ptr_;
  std::unique_ptr<udp::socket> recv_sock_ptr_;
  std::unique_ptr<deadline_timer> deadline_;
};

}  // namespace lidar
}  // namespace robosense
#endif  // PROTO_FOUND