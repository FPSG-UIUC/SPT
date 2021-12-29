/*
 * Copyright (c) 2012 ARM Limited
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
 *          Korey Sewell
 */

#ifndef __CPU_O3_ROB_IMPL_HH__
#define __CPU_O3_ROB_IMPL_HH__

#include <list>
#include <iostream>
#include <chrono>
#include <limits>

#include "cpu/o3/rob.hh"
#include "debug/Fetch.hh"
#include "debug/ROB.hh"
#include "debug/JY.hh"
#include "params/DerivO3CPU.hh"

using namespace std;

template <class Impl>
ROB<Impl>::ROB(O3CPU *_cpu, DerivO3CPUParams *params)
    : cpu(_cpu),
      numEntries(params->numROBEntries),
      squashWidth(params->squashWidth),
      numInstsInROB(0),
      numThreads(params->numThreads)
{
    std::string policy = params->smtROBPolicy;

    //Convert string to lowercase
    std::transform(policy.begin(), policy.end(), policy.begin(),
                   (int(*)(int)) tolower);

    //Figure out rob policy
    if (policy == "dynamic") {
        robPolicy = Dynamic;

        //Set Max Entries to Total ROB Capacity
        for (ThreadID tid = 0; tid < numThreads; tid++) {
            maxEntries[tid] = numEntries;
        }

    } else if (policy == "partitioned") {
        robPolicy = Partitioned;
        DPRINTF(Fetch, "ROB sharing policy set to Partitioned\n");

        //@todo:make work if part_amt doesnt divide evenly.
        int part_amt = numEntries / numThreads;

        //Divide ROB up evenly
        for (ThreadID tid = 0; tid < numThreads; tid++) {
            maxEntries[tid] = part_amt;
        }

    } else if (policy == "threshold") {
        robPolicy = Threshold;
        DPRINTF(Fetch, "ROB sharing policy set to Threshold\n");

        int threshold =  params->smtROBThreshold;;

        //Divide up by threshold amount
        for (ThreadID tid = 0; tid < numThreads; tid++) {
            maxEntries[tid] = threshold;
        }
    } else {
        assert(0 && "Invalid ROB Sharing Policy.Options Are:{Dynamic,"
                    "Partitioned, Threshold}");
    }

    resetState();
}

template <class Impl>
void
ROB<Impl>::resetState()
{
    for (ThreadID tid = 0; tid  < numThreads; tid++) {
        doneSquashing[tid] = true;
        threadEntries[tid] = 0;
        squashIt[tid] = instList[tid].end();
        squashedSeqNum[tid] = 0;
    }
    numInstsInROB = 0;

    // Initialize the "universal" ROB head & tail point to invalid
    // pointers
    head = instList[0].end();
    tail = instList[0].end();
}

template <class Impl>
std::string
ROB<Impl>::name() const
{
    return cpu->name() + ".rob";
}

template <class Impl>
void
ROB<Impl>::setActiveThreads(list<ThreadID> *at_ptr)
{
    DPRINTF(ROB, "Setting active threads list pointer.\n");
    activeThreads = at_ptr;
}

template <class Impl>
void
ROB<Impl>::drainSanityCheck() const
{
    for (ThreadID tid = 0; tid  < numThreads; tid++)
        assert(instList[tid].empty());
    assert(isEmpty());
}

template <class Impl>
void
ROB<Impl>::takeOverFrom()
{
    resetState();
}

template <class Impl>
void
ROB<Impl>::resetEntries()
{
    if (robPolicy != Dynamic || numThreads > 1) {
        int active_threads = activeThreads->size();

        list<ThreadID>::iterator threads = activeThreads->begin();
        list<ThreadID>::iterator end = activeThreads->end();

        while (threads != end) {
            ThreadID tid = *threads++;

            if (robPolicy == Partitioned) {
                maxEntries[tid] = numEntries / active_threads;
            } else if (robPolicy == Threshold && active_threads == 1) {
                maxEntries[tid] = numEntries;
            }
        }
    }
}

template <class Impl>
int
ROB<Impl>::entryAmount(ThreadID num_threads)
{
    if (robPolicy == Partitioned) {
        return numEntries / num_threads;
    } else {
        return 0;
    }
}

template <class Impl>
int
ROB<Impl>::countInsts()
{
    int total = 0;

    for (ThreadID tid = 0; tid < numThreads; tid++)
        total += countInsts(tid);

    return total;
}

template <class Impl>
int
ROB<Impl>::countInsts(ThreadID tid)
{
    return instList[tid].size();
}

template <class Impl>
void
ROB<Impl>::insertInst(DynInstPtr &inst)
{
    assert(inst);

    robWrites++;

    DPRINTF(ROB, "Adding inst PC %s to the ROB.\n", inst->pcState());

    assert(numInstsInROB != numEntries);

    ThreadID tid = inst->threadNumber;

    std::string reasonForTainting;
    auto newlyTaintedDestRegPairs = inst->getUntaintedDestRegs();

    // TAINT LIFECYCLE: destination register tainted
    if (inst->isAccess()) {
        // Access instructions always taint their destination (regardless of speculative or not)
        inst->setDestTaint(true);
        reasonForTainting = "being an access";
        if (inst->isArgsTainted()) reasonForTainting += "\n            (it also has tainted args)";
    }
    else if (inst->isArgsTainted()) {
        // Taint destination if any argument is tainted
        inst->setDestTaint(true);
        reasonForTainting = "tainted args";
    }
    else {
        inst->setDestTaint(false);
    }

    // [Rutvik, SPT] Logging stuff
    if (inst->isAccess() || inst->isArgsTainted()) {
        bool printTaintedSrcs = false;
         // [Rutvik, SPT] Inst tracking stuff
        if (cpu->isInstTracked(inst)) {
            printf("[%06lx] %lx.%lx inserted into ROB and dest tainted due to %s\n",
                (uint64_t)cpu->numCycles.value(), inst->instAddr(), inst->seqNum, reasonForTainting.c_str());
            printf("           newly tainted dest: ");
            for (auto regPair : newlyTaintedDestRegPairs) {
                printf("%d->%d(%s), ", regPair.first.index(), regPair.second->index(), regPair.first.className());
            }
            printf("\n");
            printTaintedSrcs = true;
        }

        if (inst->isArgsTainted() && printTaintedSrcs) {
            printf("           tainted srcs: ");
            for (auto regPair : inst->getTaintedSrcRegs()) {
                printf("%d->%d(%s), ", regPair.first.index(), regPair.second->index(),regPair.first.className());
            }
            printf("\n");
        }

        for (auto regPair : newlyTaintedDestRegPairs) {
            // [Rutvik, SPT] Inst tracking stuff
            for (auto trackedInst : cpu->trackedInstsForReg(regPair.second)) {
                if (trackedInst == inst) continue;
                string regType = trackedInst->containsSrcPhysReg(regPair.second) ? "src" : "dest";
                printf("[%06lx] tainting %s reg %d->%d(%s) of %lx.%lx via inserting %lx.%lx "
                        "into ROB due to %s\n",
                        (uint64_t)cpu->numCycles.value(), regType.c_str(), regPair.first.index(),
                        regPair.second->index(), regPair.first.className(), trackedInst->instAddr(),
                        trackedInst->seqNum, inst->instAddr(), inst->seqNum, reasonForTainting.c_str());
                printTaintedSrcs = true;
            }

            // [Rutvik, SPT] Reg tracking stuff
            if (cpu->isArchRegTracked(regPair.first) && !cpu->isInstTracked(inst)) {
                printf("[%06lx] tainting reg %d->%d(%s) via inserting %lx.%lx into ROB "
                       "due to %s\n",
                        (uint64_t)cpu->numCycles.value(), regPair.first.index(), regPair.second->index(),
                        regPair.first.className(), inst->instAddr(), inst->seqNum, reasonForTainting.c_str());
                printTaintedSrcs = true;
            }
        }
    }
    else {
        // [Rutvik, SPT] Inst tracking stuff
        if (cpu->isInstTracked(inst)) {
            printf("[%06lx] %lx.%lx inserted into ROB but destination not tainted\n",
                (uint64_t)cpu->numCycles.value(), inst->instAddr(), inst->seqNum);
        }
    }

    instList[tid].push_back(inst);

    //Set Up head iterator if this is the 1st instruction in the ROB
    if (numInstsInROB == 0) {
        head = instList[tid].begin();
        assert((*head) == inst);
    }

    //Must Decrement for iterator to actually be valid  since __.end()
    //actually points to 1 after the last inst
    tail = instList[tid].end();
    tail--;

    inst->setInROB();

    ++numInstsInROB;
    ++threadEntries[tid];

    assert((*tail) == inst);

    DPRINTF(ROB, "[tid:%i] Now has %d instructions.\n", tid, threadEntries[tid]);

}

template <class Impl>
void
ROB<Impl>::retireHead(ThreadID tid)
{
    robWrites++;

    assert(numInstsInROB > 0);

    // Get the head ROB instruction.
    InstIt head_it = instList[tid].begin();

    DynInstPtr head_inst = (*head_it);

    assert(head_inst->readyToCommit());

    DPRINTF(ROB, "[tid:%u]: Retiring head instruction, "
            "instruction PC %s, [sn:%lli]\n", tid, head_inst->pcState(),
            head_inst->seqNum);

    --numInstsInROB;
    --threadEntries[tid];

    head_inst->clearInROB();
    head_inst->setCommitted();

    instList[tid].erase(head_it);


    //Update "Global" Head of ROB
    updateHead();

    // @todo: A special case is needed if the instruction being
    // retired is the only instruction in the ROB; otherwise the tail
    // iterator will become invalidated.
    cpu->removeFrontInst(head_inst);
}

template <class Impl>
bool
ROB<Impl>::isHeadReady(ThreadID tid)
{
    robReads++;
    if (threadEntries[tid] != 0) {
        return instList[tid].front()->readyToCommit();
    }

    return false;
}

template <class Impl>
bool
ROB<Impl>::canCommit()
{
    //@todo: set ActiveThreads through ROB or CPU
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (isHeadReady(tid)) {
            return true;
        }
    }

    return false;
}

template <class Impl>
unsigned
ROB<Impl>::numFreeEntries()
{
    return numEntries - numInstsInROB;
}

template <class Impl>
unsigned
ROB<Impl>::numFreeEntries(ThreadID tid)
{
    return maxEntries[tid] - threadEntries[tid];
}

template <class Impl>
void
ROB<Impl>::doSquash(ThreadID tid)
{
    robWrites++;
    DPRINTF(ROB, "[tid:%u]: Squashing instructions until [sn:%i].\n",
            tid, squashedSeqNum[tid]);

    assert(squashIt[tid] != instList[tid].end());

    if ((*squashIt[tid])->seqNum < squashedSeqNum[tid]) {
        DPRINTF(ROB, "[tid:%u]: Done squashing instructions.\n",
                tid);

        squashIt[tid] = instList[tid].end();

        doneSquashing[tid] = true;
        return;
    }

    bool robTailUpdate = false;

    for (int numSquashed = 0;
         numSquashed < squashWidth &&
         squashIt[tid] != instList[tid].end() &&
         (*squashIt[tid])->seqNum > squashedSeqNum[tid];
         ++numSquashed)
    {
        DPRINTF(ROB, "[tid:%u]: Squashing instruction PC %s, seq num %i.\n",
                (*squashIt[tid])->threadNumber,
                (*squashIt[tid])->pcState(),
                (*squashIt[tid])->seqNum);

        // Mark the instruction as squashed, and ready to commit so that
        // it can drain out of the pipeline.
        (*squashIt[tid])->setSquashed();

        (*squashIt[tid])->hasPendingSquash(false);

        (*squashIt[tid])->setCanCommit();


        if (squashIt[tid] == instList[tid].begin()) {
            DPRINTF(ROB, "Reached head of instruction list while "
                    "squashing.\n");

            squashIt[tid] = instList[tid].end();

            doneSquashing[tid] = true;

            return;
        }

        InstIt tail_thread = instList[tid].end();
        tail_thread--;

        if ((*squashIt[tid]) == (*tail_thread))
            robTailUpdate = true;

        squashIt[tid]--;
    }


    // Check if ROB is done squashing.
    if ((*squashIt[tid])->seqNum <= squashedSeqNum[tid]) {
        DPRINTF(ROB, "[tid:%u]: Done squashing instructions.\n",
                tid);

        squashIt[tid] = instList[tid].end();

        doneSquashing[tid] = true;
    }

    if (robTailUpdate) {
        updateTail();
    }
}


/* **************************
 * [SafeSpec] update load insts state
 * isPrevInstsCompleted; isPrevBrsResolved
 * *************************/
template <class Impl>
void
ROB<Impl>::updateVisibleState()
{
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (instList[tid].empty())
            continue;

        InstIt inst_it = instList[tid].begin();
        InstIt tail_inst_it = instList[tid].end();

        bool prevInstsComplete = true;
        bool prevBrsResolved = true;
        bool prevInstsCommitted = true;
        bool prevBrsCommitted = true;

        while (inst_it != tail_inst_it) {
            DynInstPtr inst = *inst_it++;

            assert(inst);

            if (inst->isSquashed()) continue;

            if (!prevInstsComplete && !prevBrsResolved) {
                break;
            }

            if (prevInstsComplete)  inst->setPrevInstsCompleted();
            if (prevBrsResolved)    inst->setPrevBrsResolved();
            if (prevInstsCommitted) inst->setPrevInstsCommitted();
            if (prevBrsCommitted)   inst->setPrevBrsCommitted();

            // Update prev control insts state
            if (inst->isControl()) {
                prevBrsCommitted = false;
                if (!inst->readyToCommit() || inst->getFault() != NoFault) {
                    prevBrsResolved = false;
                }
            }

            prevInstsCommitted = false;

            // Update prev insts state
            if (inst->isNonSpeculative() || inst->isStoreConditional() || inst->isMemBarrier() ||
                inst->isWriteBarrier() || (inst->isLoad() && inst->strictlyOrdered()) )
            {
                // Some special instructions, directly set canCommit
                // when entering ROB
                prevInstsComplete = false;
            }

            if (!inst->readyToCommit() || inst->getFault() != NoFault) {
                prevInstsComplete = false;
            }

            // [Jiyong, Rutvik, SPT] Checking whether instructions have reached the VP
            if (cpu->applyDDIFT) {
                bool unsquashableBefore = inst->isUnsquashable();
                bool unsquashableNow = (!cpu->isFuturistic && inst->isPrevBrsResolved()) ||
                                       ( cpu->isFuturistic && inst->isPrevInstsCompleted());
                inst->isUnsquashable(unsquashableNow);

                if (cpu->isInstTracked(inst) && !unsquashableBefore && unsquashableNow) {
                    printf("[%06lx] %lx.%lx is now unsquashable\n", (uint64_t)cpu->numCycles.value(),
                        inst->instAddr(), inst->seqNum);
                }
            }
            else {
                inst->isUnsquashable(true);
            }
        }

        if (!cpu->disableUntaint) {
            bool trackedStuffUntainted = propagateUntaint(tid);
            if (trackedStuffUntainted && cpu->ifPrintROB) {
                printf("ROB AFTER PROPOGATING UNTAINT:\n");
                printForThread(tid, false, true);
            }
        }
    }
}


template <class Impl>
void
ROB<Impl>::updateHead()
{
    InstSeqNum lowest_num = 0;
    bool first_valid = true;

    // @todo: set ActiveThreads through ROB or CPU
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (instList[tid].empty())
            continue;

        if (first_valid) {
            head = instList[tid].begin();
            lowest_num = (*head)->seqNum;
            first_valid = false;
            continue;
        }

        InstIt head_thread = instList[tid].begin();

        DynInstPtr head_inst = (*head_thread);

        assert(head_inst != 0);

        if (head_inst->seqNum < lowest_num) {
            head = head_thread;
            lowest_num = head_inst->seqNum;
        }
    }

    if (first_valid) {
        head = instList[0].end();
    }

}

template <class Impl>
void
ROB<Impl>::updateTail()
{
    tail = instList[0].end();
    bool first_valid = true;

    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while (threads != end) {
        ThreadID tid = *threads++;

        if (instList[tid].empty()) {
            continue;
        }

        // If this is the first valid then assign w/out
        // comparison
        if (first_valid) {
            tail = instList[tid].end();
            tail--;
            first_valid = false;
            continue;
        }

        // Assign new tail if this thread's tail is younger
        // than our current "tail high"
        InstIt tail_thread = instList[tid].end();
        tail_thread--;

        if ((*tail_thread)->seqNum > (*tail)->seqNum) {
            tail = tail_thread;
        }
    }
}


template <class Impl>
void
ROB<Impl>::squash(InstSeqNum squash_num, ThreadID tid)
{
    if (isEmpty(tid)) {
        DPRINTF(ROB, "Does not need to squash due to being empty "
                "[sn:%i]\n",
                squash_num);

        return;
    }

    DPRINTF(ROB, "Starting to squash within the ROB.\n");

    robStatus[tid] = ROBSquashing;

    doneSquashing[tid] = false;

    squashedSeqNum[tid] = squash_num;

    if (!instList[tid].empty()) {
        InstIt tail_thread = instList[tid].end();
        tail_thread--;

        squashIt[tid] = tail_thread;

        doSquash(tid);
    }
}

template <class Impl>
typename Impl::DynInstPtr
ROB<Impl>::readHeadInst(ThreadID tid)
{
    if (threadEntries[tid] != 0) {
        InstIt head_thread = instList[tid].begin();

        assert((*head_thread)->isInROB());

        return *head_thread;
    } else {
        return dummyInst;
    }
}

template <class Impl>
typename Impl::DynInstPtr
ROB<Impl>::readTailInst(ThreadID tid)
{
    InstIt tail_thread = instList[tid].end();
    tail_thread--;

    return *tail_thread;
}

template <class Impl>
void
ROB<Impl>::regStats()
{
    using namespace Stats;
    robReads
        .name(name() + ".rob_reads")
        .desc("The number of ROB reads");

    robWrites
        .name(name() + ".rob_writes")
        .desc("The number of ROB writes");

    totalSquashDelayCycles
        .name(name() + ".totalSquashDelayCycles")
        .desc("The total number of cycles a squash is delayed(DDIFT)");
}

template <class Impl>
typename Impl::DynInstPtr
ROB<Impl>::findInst(ThreadID tid, InstSeqNum squash_inst)
{
    for (InstIt it = instList[tid].begin(); it != instList[tid].end(); it++) {
        if ((*it)->seqNum == squash_inst) {
            return *it;
        }
    }
    return NULL;
}

// [Rutvik, SPT] Propagate untaint forwards and backwards
template <class Impl>
bool
ROB<Impl>::propagateUntaint(ThreadID tid)
{
    bool trackedStuffUntainted = false;

    // Registers are added to a queue that has a finite limit. In terms of priority:
    //   - The regs of older instructions are preferred to those of younger instructions.
    //   - The dest regs of an instruction are preferred to the src regs
    //   - The dest and src regs are added in order of index (0, 1, 2, ...)

    // phys reg, size, offset, the inst being untainted, the method of untainting
    std::vector<std::tuple<PhysRegIdPtr, uint8_t, uint8_t, DynInstPtr, UntaintMethod>> untaintQueue;

    const int queueLimit = cpu->idealUntaint ? std::numeric_limits<int>::max() : cpu->freeParam;

    int numIters = 0;
    int numUntainted = 0;

    do {
        numIters++;

        untaintQueue.clear();

        for (auto inst : instList[tid]) {
            if (inst->isSquashed()) continue;

            bool destTainted = inst->isDestTainted();
            bool argsTainted = inst->isArgsTainted();
            bool instTracked = cpu->isInstTracked(inst);
            BitVec& destTaintBcastMask = inst->destTaintBcastMask;
            BitVec& argsTaintBcastMask = inst->argsTaintBcastMask;

            // Clear the flags for registers that are already untainted

            for (int i = 0; i < inst->numDestRegs(); i++) {
                if (!inst->isDestIdxTainted(i)) {
                    destTaintBcastMask.at(i) = false;
                }
            }

            for (int i = 0; i < inst->numSrcRegs(); i++) {
                if (!inst->isArgsIdxTainted(i)) {
                    argsTaintBcastMask.at(i) = false;
                }
            }

            // Forward untaint propagation

            if (cpu->fwdUntaint) {
                if (!inst->isTransmit() && !argsTainted && destTainted) {
                    for (int i = 0; i < inst->numDestRegs(); i++) {
                        if (inst->isDestIdxTainted(i) && !destTaintBcastMask.at(i)) {
                            destTaintBcastMask.at(i) = true;
                        }
                    }

                    // [Rutvik, SPT] Inst tracking stuff
                    if (instTracked) {
                        printf("[%06lx] %lx.%lx dest regs marked for untainting via forward propagation\n",
                            (uint64_t)cpu->numCycles.value(), inst->instAddr(), inst->seqNum);
                        printf("           marked dest regs: ");
                        auto taintedDestRegPairs = inst->getTaintedDestRegs();
                        for (auto regPair : taintedDestRegPairs) {
                            auto archReg = regPair.first;
                            auto physReg = regPair.second;
                            printf("%d->%d(%s), ", archReg.index(), physReg->index(), archReg.className());
                        }
                        printf("\n");
                    }

                    // [Rutvik, SPT] Logging stuff
                    for (int i = 0; i < inst->numDestRegs(); i++) {
                        if (!inst->isDestIdxTainted(i) || destTaintBcastMask.at(i)) continue;
                        auto archReg = inst->destRegIdx(i);
                        auto physReg = inst->renamedDestRegIdx(i);
                        // [Rutvik, SPT] Reg tracking stuff
                        if (cpu->isArchRegTracked(archReg) && !instTracked) {
                            printf("[%06lx] reg %d->%d(%s) marked for untainting via forward propagation on %lx.%lx\n",
                                    (uint64_t)cpu->numCycles.value(), archReg.index(), physReg->index(),
                                    archReg.className(), inst->instAddr(), inst->seqNum);
                        }

                        // [Rutvik, SPT] Inst tracking stuff
                        auto matchingInsts = cpu->trackedInstsForReg(physReg);
                        for (auto trackedInst : matchingInsts) {
                            if (trackedInst == inst) continue;
                            bool isSrcReg = trackedInst->containsSrcPhysReg(physReg);
                            printf("[%06lx] %s reg %d->%d(%s) of %lx.%lx marked for untainting via forward propagation "
                                    "on %lx.%lx\n",
                                    (uint64_t)cpu->numCycles.value(), (isSrcReg ? "src" : "dest"), archReg.index(),
                                    physReg->index(), archReg.className(), trackedInst->instAddr(),
                                    trackedInst->seqNum, inst->instAddr(), inst->seqNum);
                        }
                    }
                }
            }

            // Backward untaint propogation

            if (cpu->bwdUntaint) {
                string instName = inst->staticInst->getName();
                bool backwardsPassWorked = false;
                std::vector<std::pair<RegId, PhysRegIdPtr>> bwdUntaintedRegs; // Used for logging purposes
                const char* opcodes = "add,addi,adc,adci,sub,subi,sbb,sbbi";

                if (strstr(opcodes, instName.c_str()) != nullptr)
                {
                    // If an ADD or SUB has an untainted dest and all but one src is untainted, then
                    // the remaining src can be untainted
                    // NOTE: This doesn't apply if the destination is the zero reg
                    if (!inst->renamedDestRegIdx(0)->isZeroReg() &&
                        !inst->isNonCCDestTainted() &&
                        inst->numTaintedSrcRegs() == 1)
                    {
                        for (int i = 0; i < inst->numSrcRegs(); i++) {
                            if (inst->isArgsIdxTainted(i) && !argsTaintBcastMask.at(i)) {
                                argsTaintBcastMask.at(i) = true;
                                bwdUntaintedRegs.push_back({inst->srcRegIdx(i), inst->renamedSrcRegIdx(i)});
                                backwardsPassWorked = true;
                                break;
                            }
                        }
                    }
                }
                else if (instName == "mov") {
                    // If a MOV has an untainted dest then src 2 can be untainted as well
                    if (!destTainted && inst->isArgsIdxTainted(1) && !argsTaintBcastMask.at(1)) {
                        bwdUntaintedRegs.push_back({inst->srcRegIdx(1), inst->renamedSrcRegIdx(1)});
                        argsTaintBcastMask.at(1) = 1;
                        backwardsPassWorked = true;
                    }
                }

                if (backwardsPassWorked) {
                    // [Rutvik, SPT] Inst tracking stuff
                    if (instTracked) {
                        printf("[%06lx] %lx.%lx src regs marked for untainting via backward propagation\n",
                            (uint64_t)cpu->numCycles.value(), inst->instAddr(), inst->seqNum);
                        printf("           marked srcs: ");
                        for (auto regPair : bwdUntaintedRegs) {
                            auto archReg = regPair.first;
                            auto physReg = regPair.second;
                            printf("%d->%d(%s), ", archReg.index(), physReg->index(), archReg.className());
                        }
                        printf("\n");
                    }

                    // [Rutvik, SPT] Logging stuff
                    for (auto regPair : bwdUntaintedRegs) {
                        auto archReg = regPair.first;
                        auto physReg = regPair.second;
                        // [Rutvik, SPT] Reg tracking stuff
                        if (cpu->isArchRegTracked(archReg) && !instTracked) {
                            printf("[%06lx] reg %d->%d(%s) marked for untainting via backward propagation on %lx.%lx\n=",
                                    (uint64_t)cpu->numCycles.value(), archReg.index(), physReg->index(),
                                    archReg.className(), inst->instAddr(), inst->seqNum);
                        }

                        // [Rutvik, SPT] Inst tracking stuff
                        auto matchingInsts = cpu->trackedInstsForReg(physReg);
                        for (auto trackedInst : matchingInsts) {
                            if (trackedInst == inst) continue;
                            bool isSrcReg = trackedInst->containsSrcPhysReg(physReg);
                            printf("[%06lx] %s reg %d->%d(%s) of %lx.%lx marked for untainting via backward propagation "
                                    "on %lx.%lx\n",
                                    (uint64_t)cpu->numCycles.value(), (isSrcReg ? "src" : "dest"), archReg.index(),
                                    physReg->index(), archReg.className(), trackedInst->instAddr(),
                                    trackedInst->seqNum, inst->instAddr(), inst->seqNum);
                        }
                    }
                }
            }

            // Now that we've propagated the untaint, if the current instruction has any dest or src regs
            // that are waiting to be untainted, we add as many of them as we can to the queue

            assert(inst->numDestRegs() <= destTaintBcastMask.size());
            for (int i = 0; i < inst->numDestRegs(); i++) {
                if (destTaintBcastMask.at(i) && untaintQueue.size() < queueLimit) {
                    destTaintBcastMask.at(i) = false;
                    auto szAndOffs = inst->getDestRegSizeAndOffs(i);
                    untaintQueue.push_back(
                        std::make_tuple(inst->renamedDestRegIdx(i), szAndOffs.first, szAndOffs.second,
                                        inst, UntaintMethod::FwdUntaint));
                }
            }

            assert(inst->numSrcRegs() <= argsTaintBcastMask.size());
            for (int i = 0; i < inst->numSrcRegs(); i++) {
                if (argsTaintBcastMask.at(i) && untaintQueue.size() < queueLimit) {
                    argsTaintBcastMask.at(i) = false;
                    auto szAndOffs = inst->getSrcRegSizeAndOffs(i);
                    untaintQueue.push_back(
                        std::make_tuple(inst->renamedSrcRegIdx(i), szAndOffs.first, szAndOffs.second,
                                        inst, UntaintMethod::BwdUntaint));
                }
            }
        }

        for (auto t : untaintQueue) {
            auto physReg  = std::get<0>(t);
            auto size     = std::get<1>(t);
            auto offset   = std::get<2>(t);
            auto inst     = std::get<3>(t);
            auto utMethod = std::get<4>(t);

            if (cpu->readPartialTaint(physReg, size, offset)) {
                // [Rutvik, SPT] Stat collection stuff

                cpu->setUntaintMethod(physReg, utMethod);

                cpu->TotalUntaints++;
                if (utMethod == UntaintMethod::FwdUntaint) {
                    cpu->FwdUntaints++;
                }
                else if (utMethod == UntaintMethod::BwdUntaint) {
                    cpu->BwdUntaints++;
                }

                // [Rutvik, SPT] Inst tracking stuff

                if (cpu->isInstTracked(inst)) {
                    printf("[%06lx] untainting reg %d(%s) of %lx.%lx from untaint queue\n", (uint64_t)cpu->numCycles.value(),
                        physReg->index(), physReg->className(), inst->instAddr(), inst->seqNum);
                    trackedStuffUntainted = true;
                }

                for (auto trackedInst : cpu->trackedInstsForReg(physReg)) {
                    if (trackedInst == inst) continue;
                    bool isSrcReg = trackedInst->containsSrcPhysReg(physReg);
                    printf("[%06lx] untainting %s reg %d(%s) of %lx.%lx from untaint queue\n",
                            (uint64_t)cpu->numCycles.value(), (isSrcReg ? "src" : "dest"), physReg->index(),
                            physReg->className(), trackedInst->instAddr(), trackedInst->seqNum);
                    trackedStuffUntainted = true;
                }
            }

            cpu->setPartialTaint(physReg, false, size, offset);
        }

        numUntainted += untaintQueue.size();
    } while (cpu->idealUntaint && untaintQueue.size() > 0);

    return trackedStuffUntainted;
}

template <class Impl>
void
ROB<Impl>::printForThread(ThreadID tid, bool printData, bool printOnlyTrackedInsts)
{
    printf("\nROB for thread %d\n", tid);

    for(int i = 0; i < 50; i++)
        printf("-");
    printf("\n");

    for (auto instIt = instList[tid].begin(); instIt != instList[tid].end(); instIt++) {
        auto inst = (*instIt);

        // [Rutvik, SPT] Inst tracking stuff
        if (printOnlyTrackedInsts && !cpu->isInstTracked(inst)) {
            continue;
        }

        printf("ptr=%p, pc=%lx [sn:%lx], inst=%s | ", inst.get(), inst->instAddr(), inst->seqNum,
               inst->toString(printData).c_str());

        printf("fenceDelay=%d, ", inst->fenceDelay());
        printf("isTransmit=%d, ", inst->isTransmit());
        printf("isAccess=%d, ", inst->isAccess());
        printf("isMemRef=%d, ", inst->isMemRef());
        printf("argsTainted=%d, destTainted=%d, nonCCDestTainted=%d, ", inst->isArgsTainted(), inst->isDestTainted(),
               inst->isNonCCDestTainted());
        printf("pendingSquash?=%d, ", inst->hasPendingSquash());
        printf("cancommit=%d, ", inst->checkCanCommit());
        printf("status=");
        if (inst->isCommitted())
            printf("Committed, ");
        else if (inst->readyToCommit()){
            if (inst->isExecuted())
                printf("CanCommit(Exec), ");
            else
                printf("CanCommit(NonExec), ");
        }
        else if (inst->isExecuted())
            printf("Executed, ");
        else if (inst->isIssued())
            printf("Issued, ");
        else
            printf("Not Issued, ");
        printf("PBR=%d, PBC=%d, PIR=%d, PIC=%d, ", inst->isPrevBrsResolved(), inst->isPrevBrsCommitted(),
               inst->isPrevInstsCompleted(), inst->isPrevInstsCommitted());
        printf("\n");
        for(int i = 0; i < 50; i++)
            printf("-");
        printf("\n");
    }

    printf("\n");
}

template <class Impl>
void
ROB<Impl>::printForAllThreads(bool printData, bool printOnlyTrackedInsts)
{
    list<ThreadID>::iterator threads = activeThreads->begin();
    list<ThreadID>::iterator end = activeThreads->end();

    while(threads != end) {
        ThreadID tid = *threads++;
        printForThread(tid, printData, printOnlyTrackedInsts);
    }
}

template <class Impl>
typename Impl::DynInstPtr
ROB<Impl>::getResolvedPendingSquashInst(ThreadID tid)
{
    for (auto instIt = instList[tid].begin(); instIt != instList[tid].end(); instIt++) {
        auto inst = (*instIt);
        if (inst->hasPendingSquash()
            && inst->isUnsquashable()   // SPT: a delayed branch wait until it reaches VP
            && !inst->isSquashed()  // if it's already squashed, we ignore it
            ) {
            return inst;
        }
        if (inst->hasPendingSquash()
            && !inst->isSquashed()
            ) {
            ++totalSquashDelayCycles;
        }
    }
    return NULL;
}

template <class Impl>
typename Impl::DynInstPtr
ROB<Impl>::getInstFromDestReg(PhysRegIdPtr targetReg)
{
    for (auto instIt = instList[0].begin(); instIt != instList[0].end(); instIt++) {
        auto inst = *instIt;
        for(int z = 0; z < inst->numDestRegs(); z++) {
            if (targetReg == inst->renamedDestRegIdx(z)) {
                return inst;
            }
        }
    }
    return nullptr;
}

#endif//__CPU_O3_ROB_IMPL_HH__
