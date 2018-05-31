# Practical Guide to State-space Control
## Graduate-level control theory for high schoolers

I originally wrote this as a final project for an undergraduate technical writing class I took at University of California, Santa Cruz in Spring 2017 ([CMPE 185](https://cmpe185-spring17-01.courses.soe.ucsc.edu/)). It is intended as a digest of graduate-level control theory aimed at veteran FIRST Robotics Competition (FRC) students who know algebra and a bit of physics and are comfortable with the concept of a PID controller. As I learned the subject of control theory, I found that it wasn't particularly difficult, but very few resources exist outside of academia for learning it. This document is intended to rectify that situation by providing a lower the barrier to entry to the field.

This document reads a lot like a reference manual on control theory and related tools. It teaches the reader how to start designing and implementing control systems for practical systems with an emphasis on pragmatism rather than theory. While the theory is mathematically elegant at times and helps inform what is going on, one shouldn't lose sight of how it behaves when applied to real systems.

## Dependencies

* make (to run the makefile)
* texlive-core (for latexmk)
* texlive-latexextra (for bibtex and makeglossaries)
* texlive-bibtexextra (for additional BibTeX styles and bibliography databases)
* xelatex (for setting and using custom fonts)
* biber (for generating bibliography)
* Python 3.5+
  * Python Control (to generate plots and state-space results)
  * yapf (to format Python source code)
* tk (required on some Linux platforms as a backend for matplotlib)
* Inkscape (to convert SVGs to PDF)
* ghostscript (to reduce size of final PDF for publishing)

Python Control can be installed via your system's package manager or via pip. For better root locus plots, build and install Python Control from its Git repo at https://github.com/python-control/python-control instead of using 0.7.0 from pypi.python.org.

```
git clone git://github.com/python-control/python-control
pip install --user ./python-control
```

## Build

After installing the dependencies, just run `make`.

## Download

A PDF version is available at https://file.tavsys.net/control/state-space-guide.pdf.

## Future Improvements

The document is still very high level for the subject it covers as well as very
dense and fast-paced (it covers three classes of feedback control, two of which
are for graduate students, in one short document). I want to make the contents
of the document that are in the critical path more accessible. For example, one
of my readers said the linear algebra went over their head, so I need to include
more of the referenced linear algebra material in the main document (I suspect
few readers would actually go watch an hour of supplementary material on
YouTube).

What I need to do is read through this book from the perspective of explaining
it to my veteran software students, because my in-person tutor-style
explanations are generally better than what I write.

It's also not designed for skim reading (e.g., "Which equations do I need to
implement a low-fi simulator for a drivetrain?"). Calling out the final results
better would probably fix that, as it then can also be used as a quick
reference.

I'd also like to expand the introductions for each section and provide more
examples like I did for the Kalman filter design to give the reader practice
applying the skills discussed.

I should clarify the difference between linear step response and a linear
dynamical system. See
https://github.com/wpilibsuite/allwpilib/pull/1117#discussion_r191297045 for
notes on this.

The linear algebra section should be filled out with some basics that are needed
to understand the examples based on the videos already linked there.

Any other results that are good for background, but are unnecessary should be
included in an appendix.

Add graphs of zero-order hold and Euler methods of discretization. Also, answer
the question "Why is the matrix exponential used for discretization?".

A section should be included with the state-space representation chapter on
building intuition on how to fiddle with the matrices. For example, include how
C matrix augmentation doesn't affect state feedback. It's mainly to see what
different parts of the system are doing. The reference r can be passed through
with BK, control input u with K, and various states with I matrix augmentation.
Also cover what plant, controller, and observer augmentation looks like and how
that is used for u_error augmentation.

The link to the graphical introduction to Kalman filters should be replaced with
something much more comprehensive. The graphics are nice, but there isn't much
substance to promote deep understanding. I have a lot of notes from the course I
took on Kalman filters I intend to synthesize (also notes PDFs in
~/frc/state-space-guide).

The referenced derivations for the Kalman filter could be added as an appendix
since they aren't that long. Does referencing them help the document at all?

Include info from ~/frc/state-space-guide/LQR-derivs.pdf for optimal control/LQR
derivations and LQR phase margin proof.

A smoother transition is needed between the mindsets of PID control and modern
control.

* "Controls engineers have a more general framework than just PID. Here's how
  PID fits into that framework."
* Add intro to modern control part "has three parts: make model, design
  controller, estimate state. We'll go through each."
* Add section bridging gap between PID and modern control understanding "more
  general term for setpoint is reference; former focused on fiddling with
  controller params as current past and future, explain how those params are
  seen by modern control (position and velocity gains) and how bad raw integral
  control is. Explain how PID maps to modern control's states, inputs, and
  outputs.
* Modern control cares about making accurate model, then driving the states they
  care about to zero. Integral control is added with uerror if needed to handle
  model uncertainty (but prefer not to do so).

The "Implementation Steps" section needs subsections to explain how to do each
or at least examples. A small section on kinematics and dynamics in general
would be useful. The following state-space implementation examples are planned:

* Elevator (in progress)
  * Add u_error state to model
  * Include writing unit tests in Google Test
  * Include how to implement the model in C++ with Eigen
* Flywheel (in progress)
  * See elevator items
* Drivetrain
  * See elevator items
  * 971/y2017/control_loops/python/polydrivetrain.py?
* Single-jointed arm (in progress)
  * See elevator items
* Rotating claw with independent top/bottom
  * See 971/y2014/control_loops/python/claw.py
  * Use as example of coordinate transformations for states?

I should include Nx and Nu in a unified appendix on feedforwards (steady-state
feedforward section, then two-state feedforward section from existing appendix).

Include information from
https://github.com/FRC-PDR/ProgrammingDoneRight/issues/56#issue-314926154 not
otherwise mentioned in this book.

Fill out the index. This book has potential as a reference manual.

Add a section on polytopes for convex optimization?
