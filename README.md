# PRobabilistically-Informed Motion Primitives (PRIMP)
MATLAB implementation of PRobabilistically-Informed Motion Primitives, a learning-from-demonstration method on Lie group. It is published in _IEEE Transactions on Robotics (T-RO)_.

- Publication: [T-RO](), [ArXiv](https://arxiv.org/abs/2305.15761)
- Project page: [https://chirikjianlab.github.io/primp-page/](https://chirikjianlab.github.io/primp-page/)
- Python implementation is available [here](https://github.com/ChirikjianLab/primp-python).

## Authors
Sipu Ruan, Weixiao Liu, Xiaoli Wang, Xin Meng and Gregory S. Chirikjian

## Dependency
- [Robotics Toolbox (RVC 2nd edition: RTB10+MVTB4 (2017))](https://petercorke.com/toolboxes/robotics-toolbox/)
- [PbD Library](https://gitlab.idiap.ch/rli/pbdlib-matlab/)
- [Orientation-KMP](https://github.com/yanlongtu/robInfLib-matlab)

## Running instructions
### Data preparation for LfD methods
All test files are located in [`/test`](/test/) folder. To run scripts for LfD methods:

- Download the data from [Google Drive](https://drive.google.com/drive/folders/1sgfAjBgO3PWO2nCqerXjVHsovpNF4MgS?usp=sharing). All the demonstrated datasets are locataed in `/demonstrations` folder.
- Generate `/data` folder that stores all demonstration data
- Copy all the demonstration sets into the `/data` folder (Only put folders inside `/demonstrations` into `/data`)
- (Optional) To generate data trials for real-world experiments, please also download datasets from `/experiments` folder (Put the whole folder).
- Run scripts in `/test` folder

After data preparation, the structure of `/data` folder should look like
```
.
└───data
│   └───panda_arm
|   |   └───real
|   |   |   └───trajectory
|   |   |   |   └───...
|   |   └───simulation
|   |   |   └───...
│   └───lasa_handwriting
|   |   |   └───pose_data
|   |   |   |   └───...
│   └───experiments
|   |   └───...
```

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
- Plot LfD dataset [`/test/demo_lfd_dataset.m`](/test/demo_lfd_dataset.m): Plots all demonstration trajectories and store .png files in `/test` folder.

### PRIMP
- Encode demonstrations and condition on via poses [`/test/demo_primp_lfd.m`](/test/demo_primp_lfd.m)
- LfD for scooping task [`/test/demo_primp_lfd_scooping.m`](/test/demo_primp_lfd_scooping.m)
- Encode demonstrations with and without GORA [`/test/demo_primp_lfd_gora.m`](/test/demo_primp_lfd_gora.m)
- Adaptation: Pass through via poses with uncertainties [`/test/demo_primp_condition_via_poses.m`](/test/demo_primp_condition_via_poses.m)
- Adaptation: Equivariance on the change of viewing frame [`/test/demo_primp_change_view.m`](/test/demo_primp_change_view.m)
- Adaptation: Fusion with workspace density [`/test/demo_primp_lfd_fusion_wd.m`](/test/demo_primp_lfd_fusion_wd.m)

### Orientation-KMP
- Encode demonstrations and condition on via poses [`/test/demo_kmp_lfd.m`](/test/demo_kmp_lfd.m)

## Benchmark scripts
- Generate random via-point poses data for benchmarks (Required before running benchmark scripts) [`/test/generate_benchmark_trials.m`](/test/generate_benchmark_trials.m)
- Benchmark LfD for PRIMP [`/test/benchmark_lfd_primp.m`](/test/benchmark_lfd_primp.m)
- Benchmark LfD for PRIMP with storage of learned trajectory distribution [`/test/benchmark_lfd_primp_trajectory.m`](/test/benchmark_lfd_primp_trajectory.m)
- Benchmark LfD for Orientation-KMP [`/test/benchmark_lfd_kmp.m`](/test/benchmark_lfd_kmp.m)
- Benchmark for PRIMP on synthetic data between SE(3) and PCG(3) formulation [`/test/benchmark_primp_se_pcg.m`](/test/benchmark_primp_se_pcg.m)
- Benchmark for PRIMP on single demonstration [`/test/benchmark_lfd_primp_single_demo.m`](/test/benchmark_lfd_primp_single_demo.m)
- Benchmark for PRIMP on equivariant change of view [`/test/benchmark_lfd_primp_change_view.m`](/test/benchmark_lfd_primp_change_view.m)
- Qualitative comparisons among LfD methods for extrapolation cases [`/test/comparison_lfd_extrapolation.m`](/test/comparison_lfd_extrapolation.m)

## Ablation studies
- Ablation study for GORA as pre-process [`/test/ablation_primp_gora.m`](/test/ablation_primp_gora.m)
- Ablation study for fusion with robot-specific workspace density [`/test/ablation_primp_wd.m`](/test/ablation_primp_wd.m)

## Real-world experiments
- Generate reference trajectory using PRIMP for real-world experiments [`/test/generate_real_task_trials.m`](/test/generate_real_task_trials.m)

## Citation
(Coming soon)