/*
 * Copyright (c) 2012, 2014, 2017-2019, 2021 Arm Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: ST, Pomona College (2025)
 */

#ifndef __MICRO_CACHE__
#define __MICRO_CACHE__

#include "base/statistics.hh"
#include "mem/port.hh"
#include "params/MicroCache.hh"
#include "sim/sim_object.hh"

#define BLOCK_SIZE 64

namespace gem5::memory {

struct CacheBlock {
    Addr tag;
    bool valid;
    uint8_t *data;
};

class MicroCache : public SimObject
{
  private:

    // declare the ports so that we can have the call the member functions
    // of the MicroCache
    class CpuSidePort : public ResponsePort
    {
      private:
        // reference to the MicroCache using this port
        MicroCache *parent;

        // we can decide if we want to block
        bool blocked;

        // if we block an incoming request, we need to tell proc side
        // when it can resend that request --> maintain a count to
        // notify the processor of this state
        uint64_t incoming_requests_blocked;

        // to store state from responses that we cannot send to the
        // requestor yet because they are blocked
        std::list<PacketPtr> blocked_packets;

      public:
        CpuSidePort(const std::string &name, MicroCache *parent)
            : ResponsePort(name),
              parent(parent),
              blocked(false),
              incoming_requests_blocked(0)
        {  };

      protected:
        //// Packet functions ////

        // for fast forwarding
        Tick recvAtomic(PacketPtr pkt) override {
            return parent->mem_port.sendAtomic(pkt);
        };

        // for restoring from checkpoints
        void recvFunctional(PacketPtr pkt) override {
            parent->mem_port.sendFunctional(pkt);
        };

        // for timing (normal case)
        bool recvTimingReq(PacketPtr pkt) override {
            if (blocked) {
                incoming_requests_blocked++;
                return false;
            }

            if (!(pkt->isRead() || pkt->isWrite() || pkt->isInvalidate())) {
                assert(!pkt->hasData());
                assert(!pkt->needsResponse());

                parent->mem_port.sendPacket(pkt);
                return true;
            }

            blocked = true;

            parent->handleRequest(pkt);
            return true;
        };

        void recvRespRetry() override {
            while (!blocked_packets.empty() &&
                   sendTimingResp(blocked_packets.front()))
            {
                blocked_packets.pop_front();
            }
        };

        //// auxiliary functions ////

        // for gem5 on construction, get addr ranges from memory side
        AddrRangeList getAddrRanges() const override {
            return parent->mem_port.getAddrRanges();
        };

      public:
        // wrapper for sendTimingResp that handles blocking
        void sendPacket(PacketPtr pkt) {
            if (pkt->needsResponse()) {
                pkt->makeResponse();
            }

            if (pkt->isResponse() && !sendTimingResp(pkt)) {
                blocked_packets.push_back(pkt);
                return;
            }

            assert(blocked);
            blocked = false;

            if (incoming_requests_blocked > 0) {
                sendRetryReq();
                incoming_requests_blocked--;
            }
        };
    };

    class MemSidePort : public RequestPort
    {
      private:
        // reference to the MicroCache using this port
        MicroCache *parent;

        // to store state for requests that we cannot send below
        // (memory-wards) if that device is blocked
        std::list<PacketPtr> blocked_packets;

      public:
        MemSidePort(const std::string &name,
                    MicroCache *parent)
            : RequestPort(name),
              parent(parent)
        {  };

        friend class CpuSidePort; // so that we can access some of its functions

        // not in this assignment :-)
        bool isSnooping() const override { return false; };

      protected:
        //// packet functions ////

        // note: atomic and functional requests do not have responses

        // for timing (normal case)
        bool recvTimingResp(PacketPtr pkt) override {
            if (!(pkt->isRead() || pkt->isWrite())) {
                parent->sendToProc(pkt);
            } else {
                parent->handleResponse(pkt);
            }

            return true;
        };

        // when a memory device tells us it is blocked, it will notify
        // us here if it unblocks
        void recvReqRetry() override {
            while (!blocked_packets.empty() &&
                   sendTimingReq(blocked_packets.front()))
            {
                blocked_packets.pop_front();
            }
        };

        // this is a weird gem5-feature, but let's just forward the range
        // to the cpu side... we don't set assertions based on the address
        // range
        void recvRangeChange() override { parent->cpu_port.sendRangeChange(); };

      public:
        void sendPacket(PacketPtr pkt) {
            if(!sendTimingReq(pkt)) {
                blocked_packets.push_back(pkt);
            }
        };

    };

    CpuSidePort cpu_port;
    MemSidePort mem_port;

    std::list<PacketPtr> to_send;
    EventFunctionWrapper proc_send_event;

    struct CacheBlock *blk;

    // TODO: add your fields!

  public:
    MicroCache(const MicroCacheParams *p); // constructor defined in source

    // this is important for connecting the simulator front-end to back-end
    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override
    {
        if (if_name == "mem_side") {
            return mem_port;
        } else if (if_name == "cpu_side") {
            return cpu_port;
        }

        return SimObject::getPort(if_name, idx);
    };

    void handleRequest(PacketPtr pkt); // TODO: definition in source
    void handleResponse(PacketPtr pkt); // TODO: definition in source

    // utility function to create a packet to be sent around
    PacketPtr makePacket(Addr addr, MemCmd cmd) {
        RequestPtr req = std::make_shared<Request>(addr, BLOCK_SIZE, 0, 0);
        PacketPtr pkt = new Packet(req, cmd, BLOCK_SIZE);
        pkt->allocate();

        return pkt;
    };

    void sendToProc(PacketPtr pkt) {
        to_send.push_back(pkt);
        schedule(proc_send_event, curTick());
    };

    void proc_send() {
        assert(!to_send.empty());
        cpu_port.sendPacket(to_send.front());
        to_send.pop_front();
    };

    void sendToMem(PacketPtr pkt) { mem_port.sendPacket(pkt); };

    struct MicroCacheStats : public statistics::Group
    {
        const MicroCache &m;

        statistics::Scalar hits;
        statistics::Scalar misses;

        MicroCacheStats(MicroCache &m)
            : statistics::Group(&m),
              m(m),
              ADD_STAT(hits, statistics::units::Count::get(),
                       "number of hits in the micro cache"),
              ADD_STAT(misses, statistics::units::Count::get(),
                       "number of misses in the micro cache")
        {  };

        void regStats() override { statistics::Group::regStats(); };
    };

    MicroCacheStats stats;
};


}; // gem5::memory

#endif // __MICRO_CACHE__
