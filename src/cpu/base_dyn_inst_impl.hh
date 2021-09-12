/*
 * Copyright (c) 2011 ARM Limited
 * All rights reserved.
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
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
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
 * Authors: Kevin Lim
 */

#ifndef __CPU_BASE_DYN_INST_IMPL_HH__
#define __CPU_BASE_DYN_INST_IMPL_HH__

#include <iostream>
#include <iomanip>
#include <set>
#include <sstream>
#include <string>
#include <limits>

#include "base/cprintf.hh"
#include "base/trace.hh"
#include "config/the_isa.hh"
#include "cpu/base_dyn_inst.hh"
#include "cpu/exetrace.hh"
#include "cpu/op_class.hh"
#include "debug/DynInst.hh"
#include "debug/IQ.hh"
#include "mem/request.hh"
#include "sim/faults.hh"
#include "cpu/o3/regfile.hh"

template <class Impl>
BaseDynInst<Impl>::BaseDynInst(const StaticInstPtr &_staticInst,
                               const StaticInstPtr &_macroop,
                               TheISA::PCState _pc, TheISA::PCState _predPC,
                               InstSeqNum seq_num, ImplCPU *cpu)
  : staticInst(_staticInst), cpu(cpu), traceData(NULL), macroop(_macroop)
{
    seqNum = seq_num;

    pc = _pc;
    predPC = _predPC;

    initVars();
}

template <class Impl>
BaseDynInst<Impl>::BaseDynInst(const StaticInstPtr &_staticInst,
                               const StaticInstPtr &_macroop)
    : staticInst(_staticInst), traceData(NULL), macroop(_macroop)
{
    seqNum = 0;
    initVars();
}

template <class Impl>
void
BaseDynInst<Impl>::initVars()
{
    memData = NULL;
    vldData = NULL;
    effAddr = 0;
    physEffAddrLow = 0;
    physEffAddrHigh = 0;
    readyRegs = 0;
    memReqFlags = 0;

    status.reset();

    instFlags.reset();
    instFlags[RecordResult] = true;
    instFlags[Predicate] = true;
    /*** [Jiyong,DDIFT] ***/
    instFlags[HasPendingSquash] = false;
    delayedCycleCnt = 0;
    delayedAtCycle = -1;
    loadLatency = 0;

    lqIdx = -1;
    sqIdx = -1;

    fwdFromTaintedSt = false;
    stFwdData = NULL;
    stFwdDataSize = 0;
    stFwdIdx = -1;
    stFwdSeqNum = 0;
    waitForSTLPublic = false;
    isDummyLoad = false;

    // Eventually make this a parameter.
    threadNumber = 0;

    // Also make this a parameter, or perhaps get it from xc or cpu.
    asid = 0;

    // Initialize the fault to be NoFault.
    fault = NoFault;

#ifndef NDEBUG
    ++cpu->instcount;

    if (cpu->instcount > 1500) {
#ifdef DEBUG
        cpu->dumpInsts();
        dumpSNList();
#endif
        assert(cpu->instcount <= 1500);
    }

    DPRINTF(DynInst,
        "DynInst: [sn:%lli] Instruction created. Instcount for %s = %i\n",
        seqNum, cpu->name(), cpu->instcount);
#endif

#ifdef DEBUG
    cpu->snList.insert(seqNum);
#endif

    reqToVerify = NULL;
    postReq = NULL;
    postSreqLow = NULL;
    postSreqHigh = NULL;

    destTaintBcastMask = BitVec(numDestRegs(), false);
    argsTaintBcastMask = BitVec(numSrcRegs(), false);

    instFlags[ReleasedByFwdUntaint] = false;
    instFlags[ReleasedByBwdUntaint] = false;
}

template <class Impl>
BaseDynInst<Impl>::~BaseDynInst()
{
    if (memData) {
        delete [] memData;
    }

    if (stFwdData) {
        delete [] stFwdData;
    }

    if (vldData) {
        delete [] vldData;
    }

    if (traceData) {
        delete traceData;
    }

    fault = NoFault;

#ifndef NDEBUG
    --cpu->instcount;

    DPRINTF(DynInst,
        "DynInst: [sn:%lli] Instruction destroyed. Instcount for %s = %i\n",
        seqNum, cpu->name(), cpu->instcount);
#endif
#ifdef DEBUG
    cpu->snList.erase(seqNum);
#endif

    if (reqToVerify)
        delete reqToVerify;

    if (needDeletePostReq()){
        if (postReq){
            delete postReq;
            postReq = NULL;
        }
        if (postSreqLow) {
            delete postSreqLow;
            delete postSreqHigh;
            postSreqLow = NULL;
            postSreqHigh = NULL;
        }
    }
}

#ifdef DEBUG
template <class Impl>
void
BaseDynInst<Impl>::dumpSNList()
{
    std::set<InstSeqNum>::iterator sn_it = cpu->snList.begin();

    int count = 0;
    while (sn_it != cpu->snList.end()) {
        cprintf("%i: [sn:%lli] not destroyed\n", count, (*sn_it));
        count++;
        sn_it++;
    }
}
#endif

template <class Impl>
void
BaseDynInst<Impl>::dump()
{
    cprintf("T%d : %#08d `", threadNumber, pc.instAddr());
    std::cout << staticInst->disassemble(pc.instAddr());
    cprintf("'\n");
}

template <class Impl>
void
BaseDynInst<Impl>::dump(std::string &outstring)
{
    std::ostringstream s;
    s << "T" << threadNumber << " : 0x" << pc.instAddr() << " "
      << staticInst->disassemble(pc.instAddr());

    outstring = s.str();
}

template <class Impl>
void
BaseDynInst<Impl>::markSrcRegReady()
{
    DPRINTF(IQ, "[sn:%lli] has %d ready out of %d sources. RTI %d)\n",
            seqNum, readyRegs+1, numSrcRegs(), readyToIssue());
    if (++readyRegs == numSrcRegs()) {
        setCanIssue();
    }
}

template <class Impl>
void
BaseDynInst<Impl>::markSrcRegReady(RegIndex src_idx)
{
    _readySrcRegIdx[src_idx] = true;

    markSrcRegReady();
}

template <class Impl>
bool
BaseDynInst<Impl>::eaSrcsReady()
{
    // For now I am assuming that src registers 1..n-1 are the ones that the
    // EA calc depends on.  (i.e. src reg 0 is the source of the data to be
    // stored)

    for (int i = 1; i < numSrcRegs(); ++i) {
        if (!_readySrcRegIdx[i])
            return false;
    }

    return true;
}

/*** [Jiyong,DDIFT] ***/
template <class Impl>
bool
BaseDynInst<Impl>::readyToIssue_UT() const
{
    bool ret = status[CanIssue];
    if (cpu->moreTransmitInsts == 1) {
        // consider int div and fp div
        if (opClass() == IntDivOp   ||
            opClass() == FloatDivOp ||
            opClass() == FloatSqrtOp)
            ret = ret && !isArgsTainted();
    }
    else if (cpu->moreTransmitInsts == 2) {
        if (opClass() == IntDivOp ||
            isFloating())
            ret = ret && !isArgsTainted();
    }
    else {
        assert (0);
    }
    return ret;
}

template <class Impl>
std::pair<uint8_t, uint8_t>
BaseDynInst<Impl>::getDestRegSizeAndOffs(int i) const
{
    auto archReg = destRegIdx(i);
    bool isIntReg = archReg.classValue() == IntRegClass;
    bool intFoldBit = archReg.index() & X86ISA::IntFoldBit;
    uint8_t size = isIntReg ? staticInst->getDataSize() : 8;
    uint8_t offset = isIntReg && intFoldBit ? 1 : 0;
    return std::make_pair(size, offset);
}

template <class Impl>
bool
BaseDynInst<Impl>::isDestTainted() const
{
    for (int i = 0; i < numDestRegs(); i++) {
        if (isDestIdxTainted(i)) {
            return true;
        }
    }
    return false;
}

template <class Impl>
bool
BaseDynInst<Impl>::isDestIdxTainted(int i) const
{
    auto szAndOffs = getDestRegSizeAndOffs(i);
    auto size = szAndOffs.first;
    auto offs = szAndOffs.second;

    return cpu->readPartialTaint(_destRegIdx[i], size, offs);
}

template <class Impl>
const typename PhysRegFile::BitVec*
BaseDynInst<Impl>::destIdxTaintVec(int i) const
{
    return cpu->readTaintVec(_destRegIdx[i]);
}

template <class Impl>
void
BaseDynInst<Impl>::setDestTaint(bool f)
{
    for (int i = 0; i < numDestRegs(); i++) {
        auto archReg = destRegIdx(i);
        auto szAndOffs = getDestRegSizeAndOffs(i);
        bool uncondUntaint = (archReg.classValue() == IntRegClass) &&
            ((cpu->untaintTier >= 1 && archReg.index() == X86ISA::INTREG_RBP) ||
            (cpu->untaintTier >= 1 && archReg.index() == X86ISA::INTREG_RSP) ||
            (cpu->untaintTier >= 2 && archReg.index() == X86ISA::INTREG_RSI) ||
            (cpu->untaintTier >= 2 && archReg.index() == X86ISA::INTREG_RDI));
        bool newTaint = (uncondUntaint) ? false : f;
        cpu->setPartialTaint(_destRegIdx[i], newTaint, szAndOffs.first, szAndOffs.second);
    }
}

template <class Impl>
void
BaseDynInst<Impl>::setDestIdxTaintVec(int i, const BitVec& taintVec)
{
    auto archReg = destRegIdx(i);
    auto szAndOffs = getDestRegSizeAndOffs(i);
    bool uncondUntaint = (archReg.classValue() == IntRegClass) &&
        ((cpu->untaintTier >= 1 && archReg.index() == X86ISA::INTREG_RBP) ||
        (cpu->untaintTier >= 1 && archReg.index() == X86ISA::INTREG_RSP) ||
        (cpu->untaintTier >= 2 && archReg.index() == X86ISA::INTREG_RSI) ||
        (cpu->untaintTier >= 2 && archReg.index() == X86ISA::INTREG_RDI));
    if (uncondUntaint) {
        cpu->setPartialTaint(_destRegIdx[i], false, szAndOffs.first, szAndOffs.second);
    }
    else {
        cpu->setPartialTaintVec(_destRegIdx[i], taintVec, szAndOffs.first, szAndOffs.second);
    }
}

template <class Impl>
std::pair<uint8_t, uint8_t>
BaseDynInst<Impl>::getSrcRegSizeAndOffs(int i) const
{
    auto archReg = srcRegIdx(i);
    bool isIntReg = archReg.classValue() == IntRegClass;
    bool intFoldBit = archReg.index() & X86ISA::IntFoldBit;
    uint8_t offset = isIntReg && intFoldBit ? 1 : 0;
    uint8_t size = 0;
    if (isIntReg) {
        if (isLoad()) {
            size = (i == 2) ? getDestRegSizeAndOffs(0).first : staticInst->getAddrSize();
        }
        else if (isStore()) {
            size = (i == 2) ? staticInst->getDataSize() : staticInst->getAddrSize();
        }
        else {
            size = staticInst->getDataSize();
        }
    }
    else {
        size = 8;
    }
    return std::make_pair(size, offset);
}

template <class Impl>
bool
BaseDynInst<Impl>::isArgsTainted() const
{
    if (isLoad() || isStore()) return isAddrTainted();
    for (int i = 0; i < numSrcRegs(); i++) {
        if (isArgsIdxTainted(i)) {
            return true;
        }
    }
    return false;
}

template <class Impl>
bool
BaseDynInst<Impl>::isArgsIdxTainted(int i) const
{
    auto szAndOffs = getSrcRegSizeAndOffs(i);
    auto size = szAndOffs.first;
    auto offs = szAndOffs.second;

    if (isStore() && isCommitted() && i == 2) {
        const BitVec& bitVec = getSQEntry()->dataTaintVec;
        assert(offs + size <= bitVec.size());
        return std::any_of(bitVec.begin() + offs, bitVec.begin() + offs + size, [](bool b){ return b; });
    }

    return cpu->readPartialTaint(_srcRegIdx[i], size, offs);
}

template <class Impl>
const typename PhysRegFile::BitVec*
BaseDynInst<Impl>::argsIdxTaintVec(int i) const
{
    if (isStore() && isCommitted() && i == 2) return &getSQEntry()->dataTaintVec;
    return cpu->readTaintVec(_srcRegIdx[i]);
}

template <class Impl>
void
BaseDynInst<Impl>::setArgsTaint(bool f)
{
    for (int i = 0; i < numSrcRegs(); i++) {
        setArgsIdxTaint(i, f);
    }
}

template <class Impl>
void
BaseDynInst<Impl>::setArgsIdxTaint(int i, bool taint)
{
    auto archReg = srcRegIdx(i);
    auto szAndOffs = getSrcRegSizeAndOffs(i);
    auto size = szAndOffs.first;
    auto offs = szAndOffs.second;

    bool uncondUntaint = (archReg.classValue() == IntRegClass) &&
        ((cpu->untaintTier >= 1 && archReg.index() == X86ISA::INTREG_RBP) ||
        (cpu->untaintTier >= 1 && archReg.index() == X86ISA::INTREG_RSP) ||
        (cpu->untaintTier >= 2 && archReg.index() == X86ISA::INTREG_RSI) ||
        (cpu->untaintTier >= 2 && archReg.index() == X86ISA::INTREG_RDI));
    bool newTaint = (uncondUntaint) ? false : taint;

    if (isStore() && i == 2) {
        BitVec& bitVec = getSQEntry()->dataTaintVec;
        assert(size + offs <= bitVec.size());
        for (int i = 0; i < size; i++) {
            bitVec.at(i + offs) = newTaint;
        }
    }

    if (!isCommitted()) {
        cpu->setPartialTaint(_srcRegIdx[i], newTaint, size, offs);
    }
}

template <class Impl>
void
BaseDynInst<Impl>::setArgsIdxTaintVec(int i, const BitVec& taintVec)
{
    auto archReg = srcRegIdx(i);
    auto szAndOffs = getSrcRegSizeAndOffs(i);
    auto size = szAndOffs.first;
    auto offs = szAndOffs.second;

    bool uncondUntaint = (archReg.classValue() == IntRegClass) &&
        ((cpu->untaintTier >= 1 && archReg.index() == X86ISA::INTREG_RBP) ||
        (cpu->untaintTier >= 1 && archReg.index() == X86ISA::INTREG_RSP) ||
        (cpu->untaintTier >= 2 && archReg.index() == X86ISA::INTREG_RSI) ||
        (cpu->untaintTier >= 2 && archReg.index() == X86ISA::INTREG_RDI));

    if (isStore() && i == 2) {
        BitVec& bitVec = getSQEntry()->dataTaintVec;
        assert(size + offs <= bitVec.size());
        for (int i = 0; i < size; i++) {
            bitVec.at(i + offs) = uncondUntaint ? false : taintVec.at(i + offs);
        }
    }

    if (!isCommitted()) {
        if (uncondUntaint) {
            cpu->setPartialTaint(_srcRegIdx[i], false, size, offs);
        }
        else {
            cpu->setPartialTaintVec(_srcRegIdx[i], taintVec, size, offs);
        }
    }
}

template <class Impl>
bool
BaseDynInst<Impl>::isNonCCDestTainted() const
{
    for (int i = 0; i < numDestRegs(); i++) {
        if (_destRegIdx[i]->classValue() == CCRegClass) continue;
        if (isDestIdxTainted(i)) {
            return true;
        }
    }
    return false;
}

template <class Impl>
bool
BaseDynInst<Impl>::isAddrTainted() const
{
    if (!isMemRef()) return false;
    if (isCommitted()) return false;
    bool addrTainted = false;
    for (int i = 0; i < numSrcRegs(); i++) {
        if ((isStore() || (isLoad() && numSrcRegs() == 4)) && i == 2) continue;
        addrTainted |= isArgsIdxTainted(i);
    }
    return addrTainted;
}

template <class Impl>
void
BaseDynInst<Impl>::setAddrTaint(bool b)
{
    if (!isMemRef()) return;
    if (isCommitted()) return;
    for (int i = 0; i < numSrcRegs(); i++) {
        if ((isStore() || (isLoad() && numSrcRegs() == 4)) && i == 2) continue;
        setArgsIdxTaint(i, b);
        cpu->setUntaintMethod(renamedSrcRegIdx(i), UntaintMethod::ReachedVP);
    }
}

template <class Impl>
unsigned int
BaseDynInst<Impl>::numTaintedDestRegs()
{
    unsigned int counter = 0;
    for (int i = 0; i < numDestRegs(); i++) {
        if (isDestIdxTainted(i)) counter++;
    }
    return counter;
}

template <class Impl>
unsigned int
BaseDynInst<Impl>::numTaintedSrcRegs()
{
    unsigned int counter = 0;
    for (int i = 0; i < numSrcRegs(); i++) {
        if (isArgsIdxTainted(i)) counter++;
    }
    return counter;
}

template <class Impl>
unsigned int
BaseDynInst<Impl>::numTaintedAddrRegs()
{
    if (!isMemRef()) return 0;
    if (isCommitted()) return 0;
    unsigned int counter = 0;
    for (int i = 0; i < numSrcRegs(); i++) {
        if ((isStore() || (isLoad() && numSrcRegs() == 4)) && i == 2) continue;
        if (isArgsIdxTainted(i)) counter++;
    }
    return counter;
}

template <class Impl>
std::vector<std::pair<RegId, PhysRegIdPtr>>
BaseDynInst<Impl>::getTaintedSrcRegs()
{
    std::vector<std::pair<RegId, PhysRegIdPtr>> taintedRegs;
    for (int i = 0; i < numSrcRegs(); i++) {
        if (isArgsIdxTainted(i)) {
            taintedRegs.push_back(std::make_pair(srcRegIdx(i), _srcRegIdx[i]));
        }
    }
    return taintedRegs;
}

template <class Impl>
std::vector<std::pair<RegId, PhysRegIdPtr>>
BaseDynInst<Impl>::getUntaintedSrcRegs()
{
    std::vector<std::pair<RegId, PhysRegIdPtr>> untaintedRegs;
    for (int i = 0; i < numSrcRegs(); i++) {
        if (!isArgsIdxTainted(i)) {
            untaintedRegs.push_back(std::make_pair(srcRegIdx(i), _srcRegIdx[i]));
        }
    }
    return untaintedRegs;
}

template <class Impl>
std::vector<std::pair<RegId, PhysRegIdPtr>>
BaseDynInst<Impl>::getTaintedDestRegs()
{
    std::vector<std::pair<RegId, PhysRegIdPtr>> taintedRegs;
    for (int i = 0; i < numDestRegs(); i++) {
        if (isDestIdxTainted(i)) {
            taintedRegs.push_back(std::make_pair(destRegIdx(i), _destRegIdx[i]));
        }
    }
    return taintedRegs;
}

template <class Impl>
std::vector<std::pair<RegId, PhysRegIdPtr>>
BaseDynInst<Impl>::getUntaintedDestRegs()
{
    std::vector<std::pair<RegId, PhysRegIdPtr>> untaintedRegs;
    for (int i = 0; i < numDestRegs(); i++) {
        if (!isDestIdxTainted(i)) {
            untaintedRegs.push_back(std::make_pair(destRegIdx(i), _destRegIdx[i]));
        }
    }
    return untaintedRegs;
}

template <class Impl>
uint64_t
BaseDynInst<Impl>::readSrcRegIdx(int i) const
{
    auto archReg = srcRegIdx(i);
    auto physReg = _srcRegIdx[i];
    switch (archReg.classValue()) {
        case IntRegClass:
            return cpu->readIntReg(physReg);
        case FloatRegClass:
            union myUnion {
                double dValue; uint64_t iValue;
            } myValue;
            myValue.dValue = cpu->readFloatReg(physReg);
            return myValue.iValue;
        case CCRegClass:
            return cpu->readCCReg(physReg);
        case MiscRegClass:
            return cpu->readMiscRegNoEffect(physReg->index(), threadNumber);
        default:
            // TODO: consider MISC and VecElem?
            return std::numeric_limits<uint64_t>::max();
    }
}

template <class Impl>
uint64_t
BaseDynInst<Impl>::readDestRegIdx(int i) const
{
    auto archReg = destRegIdx(i);
    auto physReg = _destRegIdx[i];
    switch (archReg.classValue()) {
        case IntRegClass:
            return cpu->readIntReg(physReg);
        case FloatRegClass:
            union myUnion {
                double dValue; uint64_t iValue;
            } myValue;
            myValue.dValue = cpu->readFloatReg(physReg);
            return myValue.iValue;
        case CCRegClass:
            return cpu->readCCReg(physReg);
        case MiscRegClass:
            return cpu->readMiscRegNoEffect(physReg->index(), threadNumber);
        default:
            // TODO: consider MISC and VecElem?
            return std::numeric_limits<uint64_t>::max();
    }
}

template <class Impl>
bool
BaseDynInst<Impl>::setsCCRegs() const
{
    for (int i = 0; i < numDestRegs(); i++) {
        if (_destRegIdx[i]->classValue() == CCRegClass) {
            return true;
        }
    }
    return false;
}

template <class Impl>
std::string
BaseDynInst<Impl>::toString(bool printData) const
{
    std::stringstream ss;
    ss << staticInst->getName() << " | ";
    for (int z = 0; z < numDestRegs(); z++) {
        auto archReg = destRegIdx(z);
        auto physReg = _destRegIdx[z];
        auto szAndOffs = getDestRegSizeAndOffs(z);

        if (z > 0) {
            ss << ", ";
        }

        ss << archReg.index() << "->" << physReg->index();
        ss << "(" << archReg.className() << ")";

        auto taintVec = destIdxTaintVec(z);
        if (taintVec != nullptr) {
            ss << "[" << std::setw(2) << std::setfill('0') << std::hex
                    << PhysRegFile::taintVecAsInt(*taintVec)
                    << std::dec << "]";
            ss << "[" << std::hex << PhysRegFile::partialTaintVecAsInt(*taintVec, szAndOffs.first, szAndOffs.second) << std::dec << "]";
        }

        if (physReg->classValue() == IntRegClass) {
            ss << "{" << std::to_string(szAndOffs.first) << "}";
        }

        if (printData) {
            ss << "=" << std::hex << readDestRegIdx(z) << std::dec;
        }
    }
    ss << " | ";
    for (int z = 0; z < numSrcRegs(); z++) {
        auto archReg = srcRegIdx(z);
        auto physReg = _srcRegIdx[z];
        auto szAndOffs = getSrcRegSizeAndOffs(z);

        if (z > 0) {
            ss << ", ";
        }

        ss << archReg.index() << "->" << physReg->index();
        ss << "(" << archReg.className() << ")";

        auto taintVec = argsIdxTaintVec(z);
        if (taintVec != nullptr) {
            ss << "[" << std::setw(2) << std::setfill('0') << std::hex
                    << PhysRegFile::taintVecAsInt(*taintVec)
                    << std::dec << "]";
            ss << "[" << std::hex << PhysRegFile::partialTaintVecAsInt(*taintVec, szAndOffs.first, szAndOffs.second) << std::dec << "]";
        }

        if (physReg->classValue() == IntRegClass) {
            ss << "{" << std::to_string(szAndOffs.first) << "}";
        }

        if (printData) {
            ss << "=" << std::hex << readSrcRegIdx(z) << std::dec;
        }
    }
    return ss.str();
}

template <class Impl>
void
BaseDynInst<Impl>::fenceDelay(bool f)
{
    if (cpu->isInstTracked(instAddr())) {
        if (instFlags[ReadyToExpose] != f) {
            printf("[%06lx] %lx.%lx setting fence delay to %d\n",
                    (uint64_t)cpu->numCycles.value(), pcState().instAddr(), seqNum, f);
        }
    }
    instFlags[ReadyToExpose] = f;
}

template <class Impl>
bool
BaseDynInst<Impl>::containsDestReg(const RegId& reg)
{
    for (int i = 0; i < numDestRegs(); i++) {
        if (destRegIdx(i) == reg) return true;
    }
    return false;
}

template <class Impl>
bool
BaseDynInst<Impl>::containsSrcReg(const RegId& reg)
{
    for (int i = 0; i < numSrcRegs(); i++) {
        if (srcRegIdx(i) == reg) return true;
    }
    return false;
}

template <class Impl>
bool
BaseDynInst<Impl>::containsDestPhysReg(const PhysRegIdPtr reg)
{
    for (int i = 0; i < numDestRegs(); i++) {
        if (_destRegIdx[i] == reg) return true;
    }
    return false;
}

template <class Impl>
bool
BaseDynInst<Impl>::containsSrcPhysReg(const PhysRegIdPtr reg)
{
    for (int i = 0; i < numSrcRegs(); i++) {
        if (_srcRegIdx[i] == reg) return true;
    }
    return false;
}

#endif//__CPU_BASE_DYN_INST_IMPL_HH__
