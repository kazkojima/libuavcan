/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * Copyright (C) 2014, 2016 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <unistd.h>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <uavcan/protocol/global_time_sync_slave.hpp>
#include <uavcan/protocol/dynamic_node_id_client.hpp>
#include "board/board.hpp"

extern "C" {
void *__dso_handle __attribute__((weak)) = NULL;
// linker defined
extern char __process_stack_base__;
}

#define LOWPRIO 20
#define lowsyslog(...)

namespace app
{
namespace
{

uavcan_stm32::CanInitHelper<128> can;

constexpr unsigned NodePoolSize = 16384;

uavcan::Node<NodePoolSize>& getNode()
{
    static uavcan::Node<NodePoolSize> node(can.driver, uavcan_stm32::SystemClock::instance());
    return node;
}

void init()
{
    board::init();

    /*
     * CAN auto bit rate detection.
     * Automatic bit rate detection requires that the bus has at least one other CAN node publishing some
     * frames periodically.
     * Auto bit rate detection can be bypassed byif the desired bit rate is passed directly to can.init(), e.g.:
     *   can.init(1000000);
     */
    int res = 0;
    do
    {
        chopstx_usec_wait(1000*1000);
        lowsyslog("CAN auto bit rate detection...\n");

        std::uint32_t bitrate = 0;
        res = can.init([]() { chopstx_usec_wait(can.getRecommendedListeningDelay().toUSec()); }, bitrate);
        if (res >= 0)
        {
            lowsyslog("CAN inited at %u bps\n", unsigned(bitrate));
        }
    }
    while (res < 0);
}

class
{
    void configureNodeInfo()
    {
        getNode().setName("org.uavcan.stm32_test_stm32f446");

        /*
         * Software version
         * TODO: Fill other fields too
         */
        uavcan::protocol::SoftwareVersion swver;

        swver.vcs_commit = GIT_HASH;
        swver.optional_field_flags = swver.OPTIONAL_FIELD_FLAG_VCS_COMMIT;

        getNode().setSoftwareVersion(swver);

        lowsyslog("Git commit hash: 0x%08x\n", GIT_HASH);

        /*
         * Hardware version
         * TODO: Fill other fields too
         */
        uavcan::protocol::HardwareVersion hwver;

        std::uint8_t uid[board::UniqueIDSize] = {};
        board::readUniqueID(uid);
        std::copy(std::begin(uid), std::end(uid), std::begin(hwver.unique_id));

        getNode().setHardwareVersion(hwver);

        lowsyslog("UDID:");
        for (auto b : hwver.unique_id)
        {
            lowsyslog(" %02x", unsigned(b));
        }
        lowsyslog("\n");
    }

    void performDynamicNodeIDAllocation()
    {
        uavcan::DynamicNodeIDClient client(getNode());

        const int client_start_res = client.start(getNode().getHardwareVersion().unique_id);
        if (client_start_res < 0)
        {
            board::die(client_start_res);
        }

        lowsyslog("Waiting for dynamic node ID allocation...\n");
        while (!client.isAllocationComplete())
        {
            const int spin_res = getNode().spin(uavcan::MonotonicDuration::fromMSec(100));
            if (spin_res < 0)
            {
                lowsyslog("Spin failure: %i\n", spin_res);
            }
        }

        lowsyslog("Dynamic node ID %d allocated by %d\n",
                  int(client.getAllocatedNodeID().get()),
                  int(client.getAllocatorNodeID().get()));

        getNode().setNodeID(client.getAllocatedNodeID());
    }

public:
    void* main()
    {
        /*
         * Setting up the node parameters
         */
        configureNodeInfo();

        /*
         * Initializing the UAVCAN node
         */
        const int node_init_res = getNode().start();
        if (node_init_res < 0)
        {
            board::die(node_init_res);
        }

        /*
         * Waiting for a dynamic node ID allocation
         */
        performDynamicNodeIDAllocation();

        /*
         * Time synchronizer
         */
        static uavcan::GlobalTimeSyncSlave time_sync_slave(getNode());
        {
            const int res = time_sync_slave.start();
            if (res < 0)
            {
                board::die(res);
            }
        }

        /*
         * Main loop
         */
        lowsyslog("UAVCAN node started\n");
        getNode().setModeOperational();
        while (true)
        {
            const int spin_res = getNode().spin(uavcan::MonotonicDuration::fromMSec(5000));
            if (spin_res < 0)
            {
                lowsyslog("Spin failure: %i\n", spin_res);
            }

            lowsyslog("Time sync master: %u\n", unsigned(time_sync_slave.getMasterNodeID().get()));

            lowsyslog("Memory usage: free=%u used=%u worst=%u\n",
                      getNode().getAllocator().getNumFreeBlocks(),
                      getNode().getAllocator().getNumUsedBlocks(),
                      getNode().getAllocator().getPeakNumUsedBlocks());

            lowsyslog("CAN errors: %lu %lu\n",
                      static_cast<unsigned long>(can.driver.getIface(0)->getErrorCount()),
                      static_cast<unsigned long>(can.driver.getIface(1)->getErrorCount()));

#if !UAVCAN_TINY
            node.getLogger().setLevel(uavcan::protocol::debug::LogLevel::INFO);
            node.logInfo("app", "UTC %* sec, %* corr, %* jumps",
                         uavcan_stm32::clock::getUtc().toMSec() / 1000,
                         uavcan_stm32::clock::getUtcSpeedCorrectionPPM(),
                         uavcan_stm32::clock::getUtcAjdustmentJumpCount());
#endif
        }
        return NULL;
    }
} uavcan_node_thread;

}
}

extern "C" {
    void* main_connector(void* arg __attribute__((unused)))
    {
        return app::uavcan_node_thread.main();
    }

    static void start()
    {
        chopstx_create (LOWPRIO, (std::uint32_t)&__process_stack_base__, 8196,
                        main_connector, NULL);
    }
}

int main()
{
    app::init();

    lowsyslog("Starting the UAVCAN thread\n");
    start();

    while (true)
    {
        for (int i = 0; i < 200; i++)
        {
            board::setLed(app::can.driver.hadActivity());
            chopstx_usec_wait(25000);
        }

        const uavcan::UtcTime utc = uavcan_stm32::clock::getUtc();
        lowsyslog("UTC %lu sec   Rate corr: %fPPM   Jumps: %lu   Locked: %i\n",
                  static_cast<unsigned long>(utc.toMSec() / 1000),
                  uavcan_stm32::clock::getUtcRateCorrectionPPM(),
                  uavcan_stm32::clock::getUtcJumpCount(),
                  int(uavcan_stm32::clock::isUtcLocked()));
    }
}
