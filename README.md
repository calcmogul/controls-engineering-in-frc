# Practical Guide to State-space Control
## Graduate-level control theory for high schoolers

I originally wrote this as a final project for an undergraduate technical writing class I took at University of California, Santa Cruz in Spring 2017 ([CMPE 185](https://cmpe185-spring17-01.courses.soe.ucsc.edu/)). It is intended as a digest of graduate-level control theory aimed at veteran FIRST Robotics Competition (FRC) students who know algebra and a bit of physics and are comfortable with the concept of a PID controller. As I learned the subject of control theory, I found that it wasn't particularly difficult, but very few resources exist outside of academia for learning it. This document is intended to rectify that situation by providing a lower the barrier to entry to the field.

This document reads a lot like a reference manual on control theory and related tools. It teaches the reader how to start designing and implementing control systems for practical systems with an emphasis on pragmatism rather than theory. While the theory is mathematically elegant at times and helps inform what is going on, one shouldn't lose sight of how it behaves when applied to real systems.

## Dependencies

* make (to run the makefile)
* texlive-core (for latexmk and pdflatex)
* texlive-latexextra (for bibtex and makeglossaries)
* Python 3.5+ and Python Control (to generate plots and state-space results)
* Inkscape (to convert SVGs to PDF)

## Build

After installing the dependencies, just run `make`.

## Download

A PDF version is available at https://file.tavsys.net/control/state-space-guide.pdf.

## Future Improvements

The document is still very high level for the subject it covers as well as very dense and fast-paced (it covers three classes of feedback control, two of which are for graduate students, in one short document). I want to make the contents of the document that are in the critical path more accessible. For example, one of my readers said the linear algebra went over their head, so I need to include more of the referenced linear algebra material in the main document (I suspect few readers would actually go watch an hour of supplementary material on YouTube).

I'd also like to expand the introductions for each section and provide more examples like I did for the Kalman filter design to give the reader practice applying the skills discussed.

The linear algebra section should be filled out with some basics that are needed
to understand the examples based on the videos already linked there.

Any other results that are good for background, but are unnecessary should be included in an appendix.

The link to the graphical introduction to Kalman filters should be replaced with something much more comprehensive. The graphics are nice, but there isn't much substance to promote deep understanding. I have a lot of notes from the course I took on Kalman filters I intend to synthesize.

The referenced derivations for the Kalman filter could be added as an appendix since they aren't that long. Does referencing them help the document at all?

The "Implementation Steps" section needs subsections to explain how to do each or at least examples. A small section on kinematics and dynamics in general would be useful. The following state-space implementation examples are planned:

* Elevator (in progress)
  * Add u_error state to model
  * Include discretization and controller tuning steps in Python
  * Include writing unit tests in Google Test
  * Include how to implement the model in C++ with Eigen
* Flywheel (in progress)
  * See elevator items
* Drivetrain
  * See 971/y2017/control_loops/python/drivetrain.py
  * 971/y2017/control_loops/python/polydrivetrain.py?
* Single-jointed arm (in progress)
  * See elevator items
* Rotating claw with independent top/bottom
  * See 971/y2014/control_loops/python/claw.py
  * Use as example of coordinate transformations for states?

Add a section on polytopes for convex optimization?
