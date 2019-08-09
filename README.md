# Controls Engineering in the FIRST Robotics Competition
## Graduate-level control theory for high schoolers

I originally wrote this as a final project for an undergraduate technical
writing class I took at University of California, Santa Cruz in Spring 2017
([CMPE 185](https://cmpe185-spring17-01.courses.soe.ucsc.edu/)). It is intended
as a digest of graduate-level control theory aimed at veteran FIRST Robotics
Competition (FRC) students who know algebra and a bit of physics and are
comfortable with the concept of a PID controller. As I learned the subject of
control theory, I found that it wasn't particularly difficult, but very few
resources exist outside of academia for learning it. This book is intended to
rectify that situation by providing a lower the barrier to entry to the field.

This book reads a lot like a reference manual on control theory and related
tools. It teaches the reader how to start designing and implementing control
systems for practical systems with an emphasis on pragmatism rather than theory.
While the theory is mathematically elegant at times and helps inform what is
going on, one shouldn't lose sight of how it behaves when applied to real
systems.

## Download

A PDF version is available at
https://file.tavsys.net/control/state-space-guide.pdf.

## Running the examples

Example Python scripts can be obtained from frccontrol's Git repository at
https://github.com/calcmogul/frccontrol/tree/master/examples. Furthermore, all
scripts in the [code](code) directory are runnable by the user. They require
Python 3.5+ and frccontrol. frccontrol can be installed with the following
command.

```
pip3 install --user frccontrol
```

The scripts can be run as follows:

```
./elevator.py
```

Some Linux platforms use tk as a backend for matplotlib, so that may need to be
installed to see the plots.

## Compiling the book

After installing the dependencies, just run `make`. It will produce a PDF named
state-space-guide.pdf.

### Dependencies

To compile the book, the following packages are required.

#### Arch Linux

* base-devel (for `make` to run the makefile)
* texlive-core (for latexmk and xelatex)
* texlive-latexextra (for bibtex and makeglossaries)
* texlive-bibtexextra (for additional BibTeX styles and bibliography databases)
* biber (for generating bibliography)
* python (for generating plots)
* python-pip (for installing required Python packages)
* inkscape (to convert SVGs to PDF)
* ghostscript (to reduce size of final PDF for publishing)

#### Ubuntu

* build-essential (for `make` to run the makefile)
* latexmk
* texlive-xetex (for xelatex)
* texlive-latex-extra (for bibtex and makeglossaries)
* texlive-generic-extra (for miscellaneous LaTeX .sty files)
* texlive-bibtex-extra (for additional BibTeX styles and bibliography databases)
* xelatex (for setting and using custom fonts)
* biber (for generating bibliography)
* python3 (for generating plots)
* python3-pip (for installing required Python packages)
* inkscape (to convert SVGs to PDF)
* ghostscript (to reduce size of final PDF for publishing)

#### Python packages

These packages are installed via pip3 (e.g., `pip3 install --user frccontrol`).

* frccontrol (to provide FRC wrappers for Python Control and generate plots and
  state-space results)

The book's build process automatically sets up frccontrol and control in a venv,
so they don't have to be installed manually. Modifications to the contents of
`build/frccontrol` and `build/python-control` will be reflected in any scripts
which use the venv.

The following packages are optional because the book can compile without them.

* slycot (to generate pole-zero maps for certain state-space model examples)
* black (to format Python source code)

### Style guide

All LaTeX labels, including bibliography entries, should use underscores to
represent spaces between words instead of hyphens. Hyphens should only be used
where the character being written is already a hyphen. `.tex` file names should
use hyphens and `.py` file names should use underscores.

Glossary entries in [glossary-entries.tex](glossary-entries.tex) should be
lexographically sorted by entry key. The entry key should be the same as the
name. The words in the name should be lowercase, and the description should
start with a capital letter and end with a period. The first sentence should be
a fragment, but sentences after that, if applicable, should be complete.

Bibliography entries in [state-space-guide.bib](state-space-guide.bib) should be
sorted lexographically by label. Links to online videos should use the `@misc`
tag. Links to online static resources like PDFs should use the `@online` tag.

Book content should answer the question _why_ something works the way it does
and how and when to use it to solve problems.

## Future improvements

### Teach topics more thoroughly

The book is still somewhat dense and fast-paced (it covers three classes of
feedback control, two of which are for graduate students, in one short book).
More examples of concepts should help slow the pace down. I should read through
this book as if I was explaining it to my veteran software students and
anticipate gaps in their understanding.

Reorganize feedforward chapter
* Move plant inversion proof to derivations appendix.
* Add proof of pseudoinverse based on plant inversion proof to derivations
  appendix.

Redefine variables used at start of each chapter, or when saying "using this
thing from a previous section".

Add summary page to each chapter with relevant equations.

Add troubleshooting advice for models/impls.

Do version of linearized diff drive model that has v and omega as inputs.

Try deriving all models using Lagrangian mechanics instead. Introduce
manipulator equations?

Add executive summaries of each chapter to facilitate tl;drs similar to what has
been said in 1-on-1 discussions online when students don't understand.

Finish chapters on calculus and dynamics and part on motion planning.

Decompress walls of math. The Kalman filter derivation is a particularly big
offender.

Reintroduce variables in later chapters that came from earlier chapters to avoid
confusion and needing to trace a tree of references.

Add exercises at the end of each chapter (with solutions) for reader practice.
This book is supposed to be "practical" after all.
* This should help me see the points I want to teach in each chapter (since
  the students should be able to complete the problems) so I can make sure the
  chapter teaches it sufficiently.
* Basically like implicit learning outcomes?

Add test of P vs PD controller for flywheel
* Simulate flywheel with stochastic force applied to slow it down (simulates
  shots)
* Try P controller, PD controller, P + FF, PD + FF, and P + FF + u_error

Ramsete improvements
* Add diagram to Ramsete that shows global and vehicle coordinate frames to help
  explain what the pose represents.
* Rework robot odometry equations for use in EKF.

Add "Partial derivative" to calculus methods appendix.

Add mecanum and swerve kinematics derivations
* See ~/frc/state-space-guide/mecanum-kinematics
* Inverse kinematics derivations?
* Show going from one to the other with pseudoinverse?

Consider removing linear drivetrain model/controller in favor of nonlinear one
* The velocity part of the derivation could be folded into the nonlinear section
  because the position states were just an augmentation at the end anyway, and
  the nonlinear controller removes them in favor of global coordinates.

### Finish incomplete topics

Expand the implementation steps section on writing unit tests in Google Test.

Add examples of u_error state-space models to frccontrol and reference them in
the book. Also do C++ examples.

The following state-space implementation examples are planned:

* "Rotating claw" model with independent top/bottom
  * "This claw consists of independent upper and lower jaw pieces each driven by
    its own DC brushed motor."
  * See 971/y2014/control_loops/python/claw.py
  * Use as example of coordinate transformations for states?
  * Probably just single-jointed arm model for each side with separate LQR
    controllers, then coordinate transformation to control separation.

### Supplementary background

Any other results that are good for background but are unnecessary should be
included in an appendix.

* Add derivations for trapezoidal and S-curve profiles to the derivations
  appendix.
* Add section on controllability/observability Grammian for determining which
  states are uncontrollable/unobservable or which states are more/less
  controllable/observable.
* Add section on spline generation to complement motion profiling
  * Mention circular arc approximation
* Import an LQR derivation based on Lagrange multipliers?
* Explain N term for LQR
  * Describe implicit model following application
* Finite horizon, discrete time LQR
* Add an appendix on Ito calculus to explain where the Wiener process comes from?
* Add equations for Kalman smoother
* Add equations for and implementation of MMAE
* Add KFs for nonlinear drivetrain pose estimation
  * EKF, UKF, and comparison of the two
* Derive the two-sensor and three-sensor problems from first principles. The
  two-sensor problem uses p(x) and p(z_1|x). The three-sensor problem uses p(x),
  p(z_1|x), and p(z_2|x).
* Add a section on polytopes for convex optimization? So far, I've seen it used
  for handling saturated control inputs to prioritize tracking some states over
  others using the limited control input.
  * See 971/y2017/control_loops/python/polydrivetrain.py
  * Start with how to turn a set of constraints into a matrix equation of the
    proper form for a polytope, then how to leverage the polytope libs for
    enforcing those constraints.
* Add intro to configuration spaces and jacobians
  * See [this post](https://studywolf.wordpress.com/2013/09/07/robot-control-3-accounting-for-mass-and-gravity/)
    and previous post on jacobians
  * See [this post](https://www.chiefdelphi.com/t/multi-segement-arm-or-hilo-arm-interferences/340751/11)
    on using configuration spaces for mechanism collision avoidance

### Miscellaneous fixes

* Double check drivetrain J calculation. It doesn't include robot radius, which
  seems suspicious.
* Add note about k subscript representing the kth timestep of a variable
  * Subscripts in general denote iterations of a variable (e.g., x_1, x_2, x_n)
* Explain how equation 10.1 results in equation 10.2
* Makefile should pull frccontrol and rerun all Python scripts if new commits
  were added. If pull fails due to lack of internet, ignore it.
* Add make flag for compiling without chapter heading images
* Modify pareto_boundary.py to find and plot real Pareto boundary for LQR
  instead of using a hand-wavey approximation
* Fix bugs in dependencies
  * Fix nonsquare system pzmaps in Python Control, then enable non-Slycot pzmaps
    for remaining examples
* Make ramsete_traj.py use frccontrol's update_plant() and update_controller()
* Eventually convert the book to ConTeXt

## Licensing

This project, except for the software, is released under the Creative Commons
Attribution-ShareAlike 4.0 International license. The software is released under
the 3-clause BSD license.
