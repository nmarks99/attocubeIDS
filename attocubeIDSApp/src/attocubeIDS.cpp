#include <iostream>
#include <string_view>
#include <chrono>

#include <asynOctetSyncIO.h>
#include <epicsExport.h>
#include <epicsThread.h>
#include <iocsh.h>

#include "attocubeIDS.hpp"

static void poll_thread_C(void* pPvt) {
    AttocubeIDS* pAttocubeIDS = (AttocubeIDS*)pPvt;
    pAttocubeIDS->poll();
}

constexpr int MAX_CONTROLLERS = 1;
constexpr int INTERFACE_MASK = asynInt32Mask | asynInt64Mask | asynFloat64Mask | asynOctetMask | asynDrvUserMask;
constexpr int INTERRUPT_MASK = asynInt32Mask | asynInt64Mask | asynFloat64Mask | asynOctetMask;
constexpr int ASYN_FLAGS = ASYN_MULTIDEVICE | ASYN_CANBLOCK;

AttocubeIDS::AttocubeIDS(const char* conn_port, const char* driver_port)
    : asynPortDriver(driver_port, MAX_CONTROLLERS, INTERFACE_MASK, INTERRUPT_MASK, ASYN_FLAGS, 1, 0, 0) {

    asynStatus status = pasynOctetSyncIO->connect(conn_port, 0, &pasynUserDriver_, NULL);
    pasynOctetSyncIO->setInputEos(pasynUserDriver_, "\n", 1);
    if (status) {
        asynPrint(pasynUserDriver_, ASYN_TRACE_ERROR, "Failed to connect to Attocube IDS3010\n");
        return;
    }

    createParam(AXIS0_DISPLACEMENT_STR, asynParamInt64, &axis0DisplacementId_);
    createParam(AXIS1_DISPLACEMENT_STR, asynParamInt64, &axis1DisplacementId_);
    createParam(AXIS2_DISPLACEMENT_STR, asynParamInt64, &axis2DisplacementId_);

    epicsThreadCreate("AttocubeIDSPoller", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium), (EPICSTHREADFUNC)poll_thread_C, this);
}

asynStatus AttocubeIDS::write_read(size_t write_len) {
    nbytesout_ = 0;
    nbytesin_ = 0;
    eom_reason_ = 0;
    asynStatus status = pasynOctetSyncIO->writeRead(
	    pasynUserDriver_,
	    out_buffer_.data(), write_len,
	    in_buffer_.data(), in_buffer_.size(),
	    IO_TIMEOUT,
	    &nbytesout_, &nbytesin_, &eom_reason_);

    if (status) {
        asynPrint(pasynUserDriver_, ASYN_TRACE_ERROR, "AttocubeIDS::write_read() failed\n");
    }

    return status;
}

std::optional<json> AttocubeIDS::write_read_json(std::string_view method, json params = json{}) {
    json rpc = {
	{"jsonrpc", "2.0"},
	{"id", 1},
	{"method", method}
    };
    if (!params.empty()) rpc["params"] = params;

    // dump the json to a string, if its bigger than buffer size, return
    std::string rpc_str = rpc.dump();
    if (rpc_str.size() >= BUFFER_SIZE) {
        asynPrint(pasynUserDriver_, ASYN_TRACE_ERROR, "json out is larger that buffer size!\n");
	return std::nullopt;
    }

    // copy the json string to the output buffer
    std::copy(rpc_str.begin(), rpc_str.end(), out_buffer_.begin());

    // write the output buffer to the controller, return if there is an error
    if (write_read(rpc_str.length())) return std::nullopt;

    try {
	// parse the input JSON data and return it
	return json::parse(in_buffer_.begin(), in_buffer_.begin() + nbytesin_);
    } catch (...) {
        asynPrint(pasynUserDriver_, ASYN_TRACE_ERROR, "Error parsing input JSON\n");
	return std::nullopt;
    }
}

std::optional<std::array<int64_t, NUM_AXES>> AttocubeIDS::get_displacements() {
    if (auto data = write_read_json(Method::AxesDisplacement); data.has_value()) {
	if (data.value().contains("result")) {
	    try {
		return data.value()["result"].get<std::array<int64_t, NUM_AXES>>();
	    } catch (...) {
		return std::nullopt;
	    }
	}
    }
    return std::nullopt;
}

void AttocubeIDS::poll() {
    while (true) {
	// auto start = std::chrono::steady_clock::now();

	lock();

	if (auto disps = get_displacements(); disps) {
	    auto [d0, d1, d2] = *disps;
	    setInteger64Param(axis0DisplacementId_, d0);
	    setInteger64Param(axis1DisplacementId_, d1);
	    setInteger64Param(axis2DisplacementId_, d2);
	}

        callParamCallbacks();
	unlock();

	// auto end = std::chrono::steady_clock::now();
	// auto elap = std::chrono::duration<double>(end-start);
	// std::cout << "elap = " << elap.count()*1000 << " ms" << std::endl;
	epicsThreadSleep(0.1);
    }
}

// asynStatus AttocubeIDS::writeInt32(asynUser* pasynUser, epicsInt32 value) {
    // // int function = pasynUser->reason;
    // bool comm_ok = true;
//
    // callParamCallbacks();
    // return comm_ok ? asynSuccess : asynError;
// }
//
// asynStatus AttocubeIDS::writeFloat64(asynUser* pasynUser, epicsFloat64 value) {
    // // int function = pasynUser->reason;
    // bool comm_ok = true;
//
    // callParamCallbacks();
    // return comm_ok ? asynSuccess : asynError;
// }

// register function for iocsh
extern "C" int AttocubeIDSConfig(const char* conn_port, const char* driver_port) {
    AttocubeIDS* pAttocubeIDS = new AttocubeIDS(conn_port, driver_port);
    pAttocubeIDS = NULL;
    (void)pAttocubeIDS;
    return (asynSuccess);
}

static const iocshArg AttocubeIDSArg0 = {"Connection asyn port", iocshArgString};
static const iocshArg AttocubeIDSArg1 = {"Driver asyn port", iocshArgString};
static const iocshArg* const AttocubeIDSArgs[2] = {&AttocubeIDSArg0, &AttocubeIDSArg1};
static const iocshFuncDef AttocubeIDSFuncDef = {"AttocubeIDSConfig", 2, AttocubeIDSArgs};

static void AttocubeIDSCallFunc(const iocshArgBuf* args) {
    AttocubeIDSConfig(args[0].sval, args[1].sval);
}

void AttocubeIDSRegister(void) { iocshRegister(&AttocubeIDSFuncDef, AttocubeIDSCallFunc); }

extern "C" {
epicsExportRegistrar(AttocubeIDSRegister);
}
