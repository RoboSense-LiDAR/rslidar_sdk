/******************************************************************************
 * Copyright 2020 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#ifdef PROTO_FOUND
#include "adapter/lidar_packets_proto_adapter.h"
#define PKT_RECEIVE_BUF_SIZE 2000000
namespace robosense
{
    namespace sensor
    {
        using namespace robosense::common;

        LidarPacketsProtoAdapter::LidarPacketsProtoAdapter() : old_frmNum_(0), new_frmNum_(0)
        {
            setName("LidarPacketsProtoAdapter");
            thread_pool_ptr_ = ThreadPool::getInstance();
        }

        ErrCode LidarPacketsProtoAdapter::init(const YAML::Node &config)
        {
            setinitFlag(true);
            bool send_packets_proto;
            int msg_source = 0;
            std::string packets_send_ip;
            std::string msop_send_port;
            std::string difop_send_port;
            uint16_t msop_recv_port;
            uint16_t difop_recv_port;
            YAML::Node proto_config = yamlSubNodeAbort(config, "proto");
            yamlRead<int>(config, "msg_source", msg_source);
            yamlRead<bool>(config, "send_packets_proto", send_packets_proto, false);
            yamlReadAbort<std::string>(proto_config, "packets_send_ip", packets_send_ip);
            yamlReadAbort<std::string>(proto_config, "msop_send_port", msop_send_port);
            yamlReadAbort<std::string>(proto_config, "difop_send_port", difop_send_port);
            yamlReadAbort<uint16_t>(proto_config, "msop_recv_port", msop_recv_port);
            yamlReadAbort<uint16_t>(proto_config, "difop_recv_port", difop_recv_port);
            msop_proto_ptr_.reset(new common::ProtoBase);
            difop_proto_ptr_.reset(new common::ProtoBase);
            if (msg_source == 4)
            {
                INFO << "Receive Packets From : Protobuf-UDP" << REND;
                INFO << "Receive MSOP Scan Port: " << msop_recv_port << REND;
                INFO << "Receive DIFOP Packets Port: " << msop_recv_port << REND;
                if ((msop_proto_ptr_->initReceiver(msop_recv_port) == -1) || (difop_proto_ptr_->initReceiver(difop_recv_port) == -1))
                {
                    ERROR << "LidarPacketsReceiver: Create UDP Receiver Socket Failed OR Bind Network failed!" << REND;
                    exit(-1);
                }
                send_packets_proto = false;
            }
            if (send_packets_proto)
            {
                DEBUG << "Send Packets Through : Protobuf-UDP" << REND;
                DEBUG << "Send MSOP Scan Port: " << msop_send_port << REND;
                DEBUG << "Send DIFOP Packets Port: " << difop_send_port << REND;
                DEBUG << "Send IP: " << packets_send_ip << REND;
                if ((msop_proto_ptr_->initSender(msop_send_port, packets_send_ip) == -1) || (difop_proto_ptr_->initSender(difop_send_port, packets_send_ip) == -1))
                {
                    ERROR << "LidarPacketsReceiver: Create UDP Sender Socket Failed ! " << REND;
                    exit(-1);
                }
            }
            return ErrCode_Success;
        }

        ErrCode LidarPacketsProtoAdapter::start()
        {
            msop_buff_ = malloc(PKT_RECEIVE_BUF_SIZE);
            msop_recv_thread_.start = true;
            msop_recv_thread_.m_thread.reset(new std::thread([this]() { recvMsopPkts(); }));
            difop_recv_thread_.start = true;
            difop_recv_thread_.m_thread.reset(new std::thread([this]() { recvDifopPkts(); }));
            return ErrCode_Success;
        }
        ErrCode LidarPacketsProtoAdapter::stop()
        {
            if (msop_recv_thread_.start.load())
            {
                msop_recv_thread_.start.store(false);
                msop_recv_thread_.m_thread->join();
                free(msop_buff_);
            }
            if (difop_recv_thread_.start.load())
            {
                difop_recv_thread_.start.store(false);
                difop_recv_thread_.m_thread->join();
            }

            return common::ErrCode_Success;
        }

        void LidarPacketsProtoAdapter::send_difop(const LidarPacketMsg &msg) // Will send NavSatStatus and Odometry
        {
            difop_send_queue_.push(msg);
            if (difop_send_queue_.is_task_finished.load())
            {
                difop_send_queue_.is_task_finished.store(false);
                thread_pool_ptr_->commit([this]() { sendDifop(); });
            }
        }

        void LidarPacketsProtoAdapter::sendDifop()
        {
            while (difop_send_queue_.m_quque.size() > 0)
            {
                Proto_msg::LidarPacket proto_msg = toProtoMsg(difop_send_queue_.m_quque.front());
                if (!difop_proto_ptr_->sendSingleMsg<Proto_msg::LidarPacket>(proto_msg))
                {
                    reportError(ErrCode_LidarPacketsProtoSendError);
                }
                difop_send_queue_.pop();
            }
            difop_send_queue_.is_task_finished.store(true);
        }

        void LidarPacketsProtoAdapter::send_msop(const LidarScanMsg &msg) // Will send NavSatStatus and Odometry
        {
            msop_send_queue_.push(msg);
            if (msop_send_queue_.is_task_finished.load())
            {
                msop_send_queue_.is_task_finished.store(false);
                thread_pool_ptr_->commit([this]() { sendMsop(); });
            }
        }

        void LidarPacketsProtoAdapter::sendMsop()
        {
            while (msop_send_queue_.m_quque.size() > 0)
            {
                Proto_msg::LidarScan proto_msg = toProtoMsg(msop_send_queue_.m_quque.front());
                if (!msop_proto_ptr_->sendSplitMsg<Proto_msg::LidarScan>(proto_msg))
                {
                    reportError(ErrCode_LidarPacketsProtoSendError);
                }
                msop_send_queue_.pop();
            }
            msop_send_queue_.is_task_finished.store(true);
        }

        void LidarPacketsProtoAdapter::recvMsopPkts()
        {
            bool start_check = true;
            while (msop_recv_thread_.start.load())
            {
                void *pMsgData = malloc(MAX_RECEIVE_LENGTH);
                proto_MsgHeader tmp_header;
                int ret = msop_proto_ptr_->receiveProtoMsg(pMsgData, MAX_RECEIVE_LENGTH, tmp_header);
                if (start_check)
                {
                    if (tmp_header.msgID == 0)
                    {
                        start_check = false;
                    }
                    else
                    {
                        continue;
                    }
                }
                if (ret == -1)
                {
                    reportError(ErrCode_LidarPacketsProtoReceiveError);
                    continue;
                }
                msop_recv_queue_.push(std::make_pair(pMsgData, tmp_header));
                if (msop_recv_queue_.is_task_finished.load())
                {
                    msop_recv_queue_.is_task_finished.store(false);
                    thread_pool_ptr_->commit([&]() {
                        spliceMsopPkts();
                    });
                }
            }
        }

        void LidarPacketsProtoAdapter::spliceMsopPkts()
        {
            while (msop_recv_queue_.m_quque.size() > 0)
            {
                if (msop_recv_thread_.start.load())
                {
                    auto pair = msop_recv_queue_.m_quque.front();
                    old_frmNum_ = new_frmNum_;
                    new_frmNum_ = pair.second.frmNumber;
                    memcpy((uint8_t *)msop_buff_ + pair.second.msgID * SPLIT_SIZE, pair.first, SPLIT_SIZE);
                    if ((old_frmNum_ == new_frmNum_) && (pair.second.msgID == pair.second.totalMsgCnt - 1))
                    {
                        Proto_msg::LidarScan proto_msg;
                        proto_msg.ParseFromArray(msop_buff_, pair.second.totalMsgLen);
                        localMsopCallback(toRsMsg(proto_msg));
                    }
                }
                free(msop_recv_queue_.m_quque.front().first);
                msop_recv_queue_.pop();
            }
            msop_recv_queue_.is_task_finished.store(true);
        }

        void LidarPacketsProtoAdapter::recvDifopPkts()
        {

            while (difop_recv_thread_.start.load())
            {
                void *pMsgData = malloc(MAX_RECEIVE_LENGTH);
                proto_MsgHeader tmp_header;
                int ret = difop_proto_ptr_->receiveProtoMsg(pMsgData, MAX_RECEIVE_LENGTH, tmp_header);

                if (ret == -1)
                {
                    continue;
                }
                difop_recv_queue_.push(std::make_pair(pMsgData, tmp_header));
                if (difop_recv_queue_.is_task_finished.load())
                {
                    difop_recv_queue_.is_task_finished.store(false);
                    thread_pool_ptr_->commit([&]() {
                        spliceDifopPkts();
                    });
                }
            }
        }

        void LidarPacketsProtoAdapter::spliceDifopPkts()
        {
            while (difop_recv_queue_.m_quque.size() > 0)
            {
                if (difop_recv_thread_.start.load())
                {
                    auto pair = difop_recv_queue_.m_quque.front();
                    Proto_msg::LidarPacket protomsg;
                    protomsg.ParseFromArray(pair.first, pair.second.msgLen);
                    localDifopCallback(toRsMsg(protomsg));
                }
                free(difop_recv_queue_.m_quque.front().first);
                difop_recv_queue_.pop();
            }
            difop_recv_queue_.is_task_finished.store(true);
        }

    } // namespace sensor
} // namespace robosense
#endif // ROS_FOUND