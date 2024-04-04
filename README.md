# SLO-ECO: Single-Line-Open Aware ECO Detailed Placement and Detailed Routing Co-Optimization

This is a GitHub repository of the SLO-ECO project. The source code is forked from the OpenROAD repo, commit: [76dc13](https://github.com/The-OpenROAD-Project/OpenROAD/commit/76dc134307d935d6516eb6679dff72c6cfb13915).
The paper has been published in the ISQED 2024 conference, and presented in 4A.3 session.

### How can I compile and run this OpenROAD-integrated binary?
- Please check the OpenROAD build manual in [readthedocs#1](https://openroad.readthedocs.io/en/latest/user/Build.html).
- An example Tcl usage manual within OpenROAD binary is available in [readthedocs#2](https://openroad.readthedocs.io/en/latest/main/src/README.html).

### Where are the SLO-ECO source codes?
- Location: [src/eco/src](src/eco/src)
- Tcl commands registered to this binary (SWIG): [src/eco/src/CoreEco.i](src/eco/src/CoreEco.i)
- Top-level script to run the entire flow: [src/eco/scripts/run_gen_all.py](src/eco/scripts/run_gen_all.py)

### SMT2 formulations from [SLO-ECO paper](https://vlsicad.ucsd.edu/Publications/Conferences/406/c406.pdf) (See footnote 2 and 3)
- Commodity flow conservation (CFC)
- Vertex exclusiveness (VE)
- Edge assignment (EA)
- Metal segment (MS)
- Geometric variable (GV)
- Nub area rule (MAR)
- End-of-line (EOL)
- Objectives (Lexicographic Order)

### Code Author (C++)
- Dr. Mingyu Woo (Reimplements whole CoRe-ECO source codes based on the OpenROAD framework)

### Contributions
- Thanks to Dr. Jaehwan Kim and Prof. Andrew B. Kahng for valuable advice to proceed with this project.
- Thanks to Joong-Won Jeon and Jae-Hyun Kang for providing the detailed SLO definition.
- Thanks to previous CoRe-ECO coauthors, including Prof. C.-K. Cheng, Dr. Daeyeal Lee and Prof. Bill Lin for agreeing to open-source this SLO-ECO project.

### Citations
- J.-W. Jeon, A. B. Kahng, J.-H. Kang, J. Kim and M. Woo, "SLO-ECO: Single-Line-Open Aware ECO Detailed Placement and Detailed Routing Co-Optimization", Proc. IEEE International Symposium on Quality Electronic Design (ISQED), 2024 [(Link)](https://vlsicad.ucsd.edu/Publications/Conferences/406/c406.pdf).
- C.-K. Cheng, A. B. Kahng, I. Kang, M. Kim, D. Lee, B. Lin, D. Park and M. Woo, "CoRe-ECO: Concurrent Refinement of Detailed Place-and-Route for an Efficient ECO Automation", Proc. ACM/IEEE International Conference on Computer Design (ICCD), 2021, pp. 366-373 [(Link)](https://vlsicad.ucsd.edu/Publications/Conferences/385/c385.pdf).
