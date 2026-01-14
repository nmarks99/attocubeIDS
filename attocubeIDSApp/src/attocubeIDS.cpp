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

    createParam(RESUME_POLLER_STR, asynParamInt32, &resumePollerId_);
    createParam(SUSPEND_POLLER_STR, asynParamInt32, &suspendPollerId_);
    createParam(POLL_PERIOD_STR, asynParamFloat64, &pollPeriodId_);
    createParam(MEASUREMENT_ENABLED_STR, asynParamInt32, &measurementEnabledId_);
    createParam(AXIS0_DISPLACEMENT_STR, asynParamInt64, &axis0DisplacementId_);
    createParam(AXIS1_DISPLACEMENT_STR, asynParamInt64, &axis1DisplacementId_);
    createParam(AXIS2_DISPLACEMENT_STR, asynParamInt64, &axis2DisplacementId_);
    createParam(AXIS0_ABSOLUTE_POS_STR, asynParamInt64, &axis0AbsolutePosId_);
    createParam(AXIS1_ABSOLUTE_POS_STR, asynParamInt64, &axis1AbsolutePosId_);
    createParam(AXIS2_ABSOLUTE_POS_STR, asynParamInt64, &axis2AbsolutePosId_);
    createParam(AXIS0_REFERENCE_POS_STR, asynParamInt64, &axis0ReferencePosId_);
    createParam(AXIS1_REFERENCE_POS_STR, asynParamInt64, &axis1ReferencePosId_);
    createParam(AXIS2_REFERENCE_POS_STR, asynParamInt64, &axis2ReferencePosId_);

   poller_thread_id_ = epicsThreadCreate("AttocubeIDSPoller", epicsThreadPriorityLow,
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

std::optional<json> AttocubeIDS::write_read_json(std::string_view method, json params) {
    json rpc = {
	{"jsonrpc", "2.0"},
	{"id", 1},
	{"method", method}
    };
    if (!params.empty()) rpc["params"] = params;

    // dump the json to a string, if its bigger than buffer size, return
    std::string rpc_str = rpc.dump();
    if (rpc_str.size() >= IO_BUFFER_SIZE) {
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

void AttocubeIDS::poll() {
    while (true) {
	// auto start = std::chrono::steady_clock::now();

	lock();

	double poll_period;
	getDoubleParam(pollPeriodId_, &poll_period);
	poll_period = std::max(poll_period, POLL_PERIOD_MIN);

	if (auto disps = do_rpc<I64Array4>(Method::AxesDisplacement); disps) {
	    auto [_, d0, d1, d2] = *disps;
	    setInteger64Param(axis0DisplacementId_, d0);
	    setInteger64Param(axis1DisplacementId_, d1);
	    setInteger64Param(axis2DisplacementId_, d2);
	}

	if (auto abspos = do_rpc<I64Array4>(Method::AbsolutePositions); abspos) {
	    auto [_, p0, p1, p2] = *abspos;
	    setInteger64Param(axis0AbsolutePosId_, p0);
	    setInteger64Param(axis1AbsolutePosId_, p1);
	    setInteger64Param(axis2AbsolutePosId_, p2);
	}

	if (auto refpos = do_rpc<I64Array4>(Method::ReferencePositions); refpos) {
	    auto [_, r0, r1, r2] = *refpos;
	    setInteger64Param(axis0ReferencePosId_, r0);
	    setInteger64Param(axis1ReferencePosId_, r1);
	    setInteger64Param(axis2ReferencePosId_, r2);
	}

	if (auto meas_enabled = do_rpc<IntPair>(Method::MeasurementEnabled); meas_enabled) {
	    auto [_, enabled] = *meas_enabled;
	    setIntegerParam(measurementEnabledId_, enabled);
	}

        callParamCallbacks();
	unlock();

	// auto end = std::chrono::steady_clock::now();
	// auto elap = std::chrono::duration<double>(end-start);
	// std::cout << "elap = " << elap.count()*1000 << " ms" << std::endl;
	if (poller_should_suspend_) {
	    std::cout << "Poller: Suspending...\n";
	    poller_should_suspend_ = false;
	    epicsThreadSuspendSelf();
	}

	epicsThreadSleep(poll_period);
    }
}

asynStatus AttocubeIDS::writeInt32(asynUser* pasynUser, epicsInt32 value) {
    int function = pasynUser->reason;
    bool comm_ok = true;

    if (function == resumePollerId_) {
	epicsThreadResume(poller_thread_id_);
    } else if (function == suspendPollerId_) {
	poller_should_suspend_ = true;
    }

    callParamCallbacks();
    return comm_ok ? asynSuccess : asynError;
}

// asynStatus AttocubeIDS::writeFloat64(asynUser* pasynUser, epicsFloat64 value) {
    // // int function = pasynUser->reason;
    // bool comm_ok = true;
//
    // callParamCallbacks();
    // return comm_ok ? asynSuccess : asynError;
// }

// register function for iocsh
extern "C" int AttocubeIDSConfig(const char* conn_port, const char* driver_port) {
    new AttocubeIDS(conn_port, driver_port);
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
