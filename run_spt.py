import os
import sys
import argparse

parser = argparse.ArgumentParser()

parser.add_argument('--executable', required=True)
parser.add_argument('--enable-spt', action='store_true')
parser.add_argument('--threat-model',
    choices=['spectre', 'futuristic'],
    required='--enable-spt' in sys.argv)
parser.add_argument('--untaint-method',
    choices=['none', 'fwd', 'bwd', 'ideal'],
    required='--enable-spt' in sys.argv)
mutex_group = parser.add_mutually_exclusive_group()
mutex_group.add_argument('--enable-shadow-l1', action='store_true')
mutex_group.add_argument('--enable-shadow-mem', action='store_true')
parser.add_argument('--track-insts', action='store_true')
parser.add_argument('--output-dir')

args = parser.parse_args()

if not os.path.isfile(args.executable):
    parser.error('argument to --executable must be a file that exists!')
if args.track_insts and not args.enable_spt:
    parser.error('can\'t specifiy --track-insts if --enable-spt isn\'t specified!')
apply_ddift = '1' if args.enable_spt else '0'
if not args.enable_spt:
    threat_scheme = 'UnsafeBaseline'
else:
    if args.threat_model == 'spectre':
        threat_scheme = 'SpectreSafeFence'
    else:
        threat_scheme = 'FuturisticSafeFence'
disable_untaint = '1' if args.untaint_method == 'none' else '0'
fwd_untaint = '1' if (args.untaint_method in ['fwd', 'bwd', 'ideal']) else '0'
bwd_untaint = '1' if (args.untaint_method in ['bwd', 'ideal']) else '0'
ideal_untaint = '1' if args.untaint_method == 'ideal' else '0'
enable_shadow_l1 = '1' if args.enable_shadow_l1 or args.enable_shadow_mem else '0'
bottomless_shadow_l1 = '1' if args.enable_shadow_mem else '0'
track_insts_arg = '--trackInstsFile=all' if args.track_insts else ''

spt_dir = os.path.dirname(os.path.abspath(__file__))

outdir = args.output_dir if args.output_dir is not None else spt_dir+'/m5out'

COMMAND_STR = '''
{spt_dir}/build/X86_MESI_Two_Level/gem5.fast --outdir={outdir} {spt_dir}/configs/example/se.py -c {executable} --num-cpus=1 --mem-size=4GB --num-l2caches=1 --l1d_assoc=8 --l2_assoc=16 --l1i_assoc=4 --cpu-type=DerivO3CPU --needsTSO=1 --scheme={threat_scheme} --num-dirs=1 --ruby --applyDDIFT={apply_ddift} --configImpFlow=Lazy --moreTransmitInsts=0 --ifPrintROB=0 --network=simple --topology=Mesh_XY --mesh-rows=1 --printDelayCycles=0 --enableShadowL1={enable_shadow_l1} --bottomlessShadowL1={bottomless_shadow_l1} --untaintTier=0 --freeParam=3 --disableUntaint={disable_untaint} --fwdUntaint={fwd_untaint} --bwdUntaint={bwd_untaint} --idealUntaint={ideal_untaint} {track_insts_arg}
'''.format(spt_dir=spt_dir, executable=args.executable, threat_scheme=threat_scheme, apply_ddift=apply_ddift,
  enable_shadow_l1=enable_shadow_l1, bottomless_shadow_l1=bottomless_shadow_l1, disable_untaint=disable_untaint,
  fwd_untaint=fwd_untaint, bwd_untaint=bwd_untaint, ideal_untaint=ideal_untaint, track_insts_arg=track_insts_arg,
  outdir=outdir)

os.system(COMMAND_STR)
