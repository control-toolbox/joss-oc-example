# JOSS paper example — OptimalControl.jl

This repository provides a standalone illustrative example of
[OptimalControl.jl](https://github.com/control-toolbox/OptimalControl.jl),
serving as a companion annex to the submitted JOSS paper:

> **OptimalControl.jl: A Julia package for modeling and solving optimal control problems with ODEs**
> J.-B. Caillau, O. Cots, J. Gergaud, P. Martinon, S. Sed.

The script [`example.jl`](./example.jl) solves a constrained energy
minimization problem, combining a **direct method** (transcription to an NLP
solved with IPOPT on a coarse grid) with an **indirect method** (shooting
based on Pontryagin's Maximum Principle) to compute a high-accuracy
solution. The direct solution is used to detect the three-arc structure
(unconstrained–constrained–unconstrained) and to initialize the shooting.

The generated figure [`comparison.pdf`](./comparison.pdf) compares both
solutions.

## Requirements

- [Julia](https://julialang.org) ≥ 1.10.

The Julia dependencies and their compatible versions are specified in
[`Project.toml`](./Project.toml) (via the `[compat]` section) and are
installed automatically when running the script.

## Usage

From this directory:

```sh
julia example.jl
```

The script activates the local environment, instantiates the dependencies,
solves the problem with both the direct and indirect methods, and writes
`comparison.pdf` with the comparison plot.
<!-- 
## Citation

If you use this example or OptimalControl.jl in your work, please cite the
JOSS paper (DOI to be added upon publication):

```bibtex
@article{OptimalControl_jl_JOSS,
  author  = {Caillau, Jean-Baptiste and Cots, Olivier and Gergaud, Joseph
             and Martinon, Pierre and Sed, Sophia},
  title   = {OptimalControl.jl: A Julia package for modeling and solving
             optimal control problems with ODEs},
  journal = {Journal of Open Source Software},
  year    = {2026},
  doi     = {TBA}
}
``` -->

## License

MIT — see the [OptimalControl.jl repository](https://github.com/control-toolbox/OptimalControl.jl)
for the license of the underlying package.
