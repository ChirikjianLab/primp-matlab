# PRobabilistically-Informed Motion Primitives (PRIMP)
MATLAB implementation of PRobabilistically-Informed Motion Primitives, a learning-from-demonstration method on Lie group.

Documentation is available in [ArXiv](https://arxiv.org/abs/2305.15761).

Python implementation is available [here](https://github.com/ChirikjianLab/primp-python).

## Authors
Sipu Ruan (repository maintainer), Weixiao Liu, Xiaoli Wang, Xin Meng and Gregory S. Chirikjian

## Dependency
- [Robotics Toolbox (RVC 2nd edition: RTB10+MVTB4 (2017))](https://petercorke.com/toolboxes/robotics-toolbox/)
- [PbD Library](https://gitlab.idiap.ch/rli/pbdlib-matlab/)
- [Orientation-KMP](https://github.com/yanlongtu/robInfLib-matlab)

## Running instructions
### Data preparation for LfD methods
All test files are located in [`/test`](/test/) folder. To run scripts for LfD methods:

- Download the data from [Google Drive](https://drive.google.com/drive/folders/1sgfAjBgO3PWO2nCqerXjVHsovpNF4MgS?usp=sharing). All the demonstrated datasets are locataed in `/demonstrations` folder.
- Generate `/data` folder that stores all demonstration data
- Copy all the demonstration sets into the `/data` folder
- Run scripts in `/test` folder

### Source code for Orientation-KMP method
To run Orientation-KMP method,

- Clone from [Orientation-KMP Repository](https://github.com/yanlongtu/robInfLib-matlab)
- Add correct paths of functions
```
addpath path-prefix/pbdlib-matlab/demos/m_fcts/
addpath path-prefix/robInfLib-matlab/fcts/
```

### Mex DTW C-code
DTW is required to evaluate method performance. The source C code is [`/src/util/dtw_c.c`](/src/util/dtw_c.c), which needs to be generated as a `.mex` file.

- Go to `/src/util`
- Mex the C code for DTW
```
mex dtw_c.c
```

## Demonstration scripts
### Dataset
- Plot LfD dataset [`/test/demo_lfd_dataset.m`](/test/demo_lfd_dataset.m)

### PRIMP
- Encode demonstrations and condition on via poses [`/test/demo_primp_lfd.m`](/test/demo_primp_lfd.m)
- LfD for scooping task [`/test/demo_primp_lfd_scooping.m`](/test/demo_primp_lfd_scooping.m)
- Encode demonstrations with and without GORA [`/test/demo_primp_lfd_gora.m`](/test/demo_primp_lfd_gora.m)
- Adaptation: Pass through via poses with uncertainties [`/test/demo_primp_condition_via_poses.m`](/test/demo_primp_condition_via_poses.m)
- Adaptation: Equivariance on the change of viewing frame [`/test/demo_primp_change_view.m`](/test/demo_primp_change_view.m)
- Adaptation: Fusion with workspace density [`/test/demo_primp_lfd_fusion_wd.m`](/test/demo_primp_lfd_fusion_wd.m)

### Orientation-KMP
- ncode demonstrations and condition on via poses [`/test/demo_kmp_lfd.m`](/test/demo_kmp_lfd.m)

## Benchmark scripts
- Benchmark LfD for PRIMP [`/test/benchmark_lfd_primp.m`](/test/benchmark_lfd_primp.m)
- Benchmark LfD for PRIMP with storage of learned trajectory distribution [`/test/benchmark_lfd_primp_trajectory.m`](/test/benchmark_lfd_primp_trajectory.m)
- Benchmark for PRIMP on synthetic data between SE(3) and PCG(3) formulation [`/test/benchmark_primp_se_pcg.m`](/test/benchmark_primp_se_pcg.m)
- Benchmark LfD for Orientation-KMP [`/test/benchmark_lfd_kmp.m`](/test/benchmark_lfd_kmp.m)
- Qualitative comparisons among LfD methods for extrapolation cases [`/test/benchmark_lfd_extrapolation.m`](/test/benchmark_lfd_extrapolation.m)
