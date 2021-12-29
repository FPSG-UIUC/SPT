# Evaluation Code for Speculative Privacy Tracking (SPT)

### Rutvik Choudhary<sup>&#10038;</sup>, Jiyong Yu<sup>&#10038;</sup>, Chris Fletcher<sup>&#10038;</sup>, Adam Morrison<sup>†</sup>

<sup>&#10038;</sup> UIUC, USA&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<sup>†</sup> Tel Aviv University, Israel

[![DOI](https://zenodo.org/badge/405761413.svg)](https://zenodo.org/badge/latestdoi/405761413)

## Intro

This is the code we used to do the evaluations for our MICRO 2021 paper: _Speculative Privacy Tracking (SPT): Leaking Information From Speculative Execution Without Compromising Privacy_.

## Citation

```
@inproceedings{SPT2021,
  title = {Speculative Privacy Tracking (SPT): Leaking Information From Speculative Execution Without Compromising Privacy},
  author = {Rutvik Choudhary and Jiyong Yu and Christopher W. Fletcher and Adam Morrison},
  booktitle = {Proceedings of the 54th Annual IEEE/ACM International Symposium on Microarchitecture},
  year = {2021},
}
```

## Cloning

`cd` to wherever you want your copy of the repo to live. Then run

```
git clone https://github.com/FPSG-UIUC/SPT.git
```

**From now on, we will use `$SPT` to refer to the filepath of your local copy of the repo**

## Building

### Requirements

Tool | Version | Notes
--- | --- | ---
Ubuntu | 16.04 | :warning: Unfortunately the repo doesn't appear to run correctly when tested on Ubuntu 20.04 (we haven't tested Ubuntu 18). If you run into a similar issue, we suggest using a Docker container that has the correct version of Ubuntu.
Python | 2.7 | This is needed to build the repo. You won't invoke this manually.
Python | 3.5+ | You will manually invoke this to run the helper script.
SCons | 2.5.1 | :warning: SCons versions 3 and beyond will use Python 3 under the hood and will cause build failures, so **make sure you use this version!** It can't be found with Anaconda, but it _can_ be found with pip.
g++ | 5.3.1 | This version is default installed when `apt install g++` is run on Ubuntu 16.04

There might be one or two extra dependencies that aren't already on your machine, but the gem5 build scripts will let you know what is missing, and from there it's just a matter of `apt install`.

### Build Command

`cd` into `$SPT` and run

```
scons build/X86_MESI_Two_Level/gem5.fast -j <num cores>
```

## Running

Running the gem5 executable directly is a bit complicated since there are some legacy command-line options from previous projects. Thus it is **highly** recommended that you use the helper script `run_spt.py`. **You need to run it with Python version 3.5 or later!**

**Note:** If you want to see the actual command that is run, gem5 prints it near the start.

**Note:** Our helper script targets syscall emulation mode for gem5. For full system mode, you're on your own unfortunately.

### Configuration Options for Helper Script

Parameter | Values | Description | Requirements/Restrictions
--- | --- | --- | ---
`--executable` | Filesystem Path | The executable you want to run with gem5 | Required
`--enable-spt` | n/a | Enables SPT's protection mechanism (if this is left out then then you have the `UnsafeBaseline`) | Not required
`--threat-model` | `spectre`, `futuristic` | Which threat model to simulate under (see paper for details) | Required if `--enable-spt` is specified
`--untaint-method` | `none`, `fwd`, `bwd`, `ideal` | What type of untaint propagation to allow (see paper for details) | Required if `--enable-spt` is specified
`--enable-shadow-l1` | n/a | Enables the shadow L1 (aka taint tracking through the L1 cache) | Cannot be specified if `--enable-shadow-mem` was specified
`--enable-shadow-mem` | n/a | Enables shadow memory (aka taint tracking  through all of memory) | Cannot be specified if `--enable-shadow-l1` was specified

### Miscellaneous Options for Helper Script

Parameter | Values | Description | Requirements/Restrictions
--- | --- | --- | --- |
`--track-insts` | n/a | If this is specified, then gem5 will output detailed taint tracking information. This is recommended if you wish to observe the flow of taint/untaint through your program. Note that a _lot_ of output will be produced! | Can only be specified if `--enable-spt` is specified
`--output-dir` | Filesystem Path | This is the directory where the output `stats.txt` file will be generated. Note that whatever directory you specify here will be created. If unspecified, then it will create a directory `m5out` in `$SPT` and put `stats.txt` there. | Not required


### Running Configurations from the Paper

Listed below are the configurations that we evaluated in our paper and the corresponding parameters to `run_spt.py`. We have omitted the following parameters in the table since their values are user-dependent:

  * `--executable`
  * `--threat-model`
  * `--track-insts`

Configuration from Paper | Parameters
--- | ---
SecureBaseline | `--enable-spt --untaint-method=none`
Fwd, NoShadowL1 | `--enable-spt --untaint-method=fwd`
Bwd, NoShadowL1 | `--enable-spt --untaint-method=bwd`
Bwd, ShadowL1 | `--enable-spt --untaint-method=bwd --enable-shadow-l1`
Bwd, ShadowMem | `--enable-spt --untaint-method=bwd --enable-shadow-mem`
Ideal, ShadowMem | `--enable-spt --untaint-method=ideal --enable-shadow-mem`

To run the InsecureBaseline, provide _only_ the `--executable` parameter.

## Getting the Results

When gem5 is run, it will create a file called `stats.txt` in the directory you specified with `--output-dir`. If you _didn't_ specify `--output-dir` then `stats.txt` will be in `$SPT/m5out`.

In this file are many statistics, though the one of most interest will be `numCycles`, which specifies how many cycles the program took to execute. There many other statistics provided by gem5 that have comments next to them briefly describing what they are.

On top of the statistics provided by gem5, we provide some custom statistics as well:

Statistic | Description
--- | ---
TotalUntaints          | Every time a register goes from tainted to untainted
VPUntaints             | Secret-dependent operand register untainted because a transmit reached the visibility point
FwdUntaints            | Register untainted because of forward untaint propagation
BwdUntaints            | Register untainted because of backward untaint propagation
SL1Untaints            | Load destination register untainted because of the shadow L1
DelayedSL1Untaints     | Load destination register untainted because of the shadow L1 (but had to wait until `STLPublic` was true)
STLFwdUntaints         | Load destination register untainted because of store-to-load forwarding
STLBwdUntaints         | Store source register untainted because of store-to-load forwarding
DelayedSTLFwdUntaints  | Load destination register untainted because of store-to-load forwarding (but had to wait until `STLPublic` was true)
DelayedSTLBwdUntaints  | Store source register untainted because of store-to-load forwarding (but had to wait until `STLPublic` was true)
SL1UntaintedHit        | A hit in the shadow L1 that returns untainted data
SL1TaintedHit          | A hit in the shadow L1 that returns tainted data
DelayedSL1UntaintedHit | A hit in the shadow L1 that returns untainted data (but had to wait until `STLPublic` was true)
DelayedSL1TaintedHit   | A hit in the shadow L1 that returns tainted data (but had to wait until `STLPublic` was true)
SL1Miss                | A miss in the shadow L1 (which always returns tainted data)
DelayedSL1Miss         | A miss in the shadow L1 (which always returns tainted data, had to wait until `STLPublic` was true)

For more details/explanation on these events, see our paper.

## Browsing the Code

If you want to look for our specific additions, a good string to search for is `Rutvik, STT+`. That should indicate where most of the changes are. There may be some others that can be seen, but these locations will serve as a good starting point.

