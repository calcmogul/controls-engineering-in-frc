# TODO

## Teach topics more thoroughly

The book is still somewhat dense and fast-paced (it covers three classes of
feedback control, two of which are for graduate students, in one short book).
More examples of concepts should help slow the pace down. I should read through
this book as if I was explaining it to my veteran software students and
anticipate gaps in their understanding.

Add stubs for missing chapters, missing sections, and todo items.

### All chapters

* Fix book glossary (less self-referential?) and define terms on first use
  * Box at beginning of each chapter with terms?
* Redefine variables used at start of each chapter, or when saying "using this
  thing from a previous section".
* Add summary page to each chapter with relevant equations.
* Add executive summaries of each chapter to facilitate tl;drs similar to what
  has been said in 1-on-1 discussions online when students don't understand.
* Decompress walls of math. The Kalman filter derivation is a particularly big
  offender.
* Reintroduce variables in later chapters that came from earlier chapters to
  avoid confusion and needing to trace a tree of references.
* Add exercises at the end of each chapter (with solutions) for reader practice.
  This book is supposed to be "practical" after all.
  * This should help me see the points I want to teach in each chapter (since
    the students should be able to complete the problems) so I can make sure the
    chapter teaches it sufficiently.
  * Basically like implicit learning outcomes?

### State-space controllers

* Add test of P vs PD controller for flywheel
  * Simulate flywheel with stochastic force applied to slow it down (simulates
    shots)
  * Try P controller, PD controller, P + FF, PD + FF, and P + FF + u_error
* Add information on controllability and observability Grammians
  * Link to controllability Grammian video by Steve Brunton
* Add links to implementations
* Mention https://en.wikipedia.org/wiki/Separation_principle

### State-space model examples

* Add examples of input error state-space models to frccontrol and reference
  them in the book. Also do C++ examples.

### Nonlinear control

* Ramsete improvements
  * Add diagram to Ramsete that shows global and vehicle coordinate frames to
    help explain what the pose represents.

### System modeling

* Manipulator equations and Jacobians
  * Try deriving all models using Lagrangian mechanics instead and introduce
    manipulator equations
    * See [this post](https://studywolf.wordpress.com/2013/09/07/robot-control-3-accounting-for-mass-and-gravity/)
      and previous post on Jacobians
* Add troubleshooting advice for models/impls.
* Finish chapter on dynamics.

## Supplementary background

### State-space controllers

* Add section on controllability/observability Grammian for determining which
  states are uncontrollable/unobservable or which states are more/less
  controllable/observable.
* Finish implicit model following
  * Add implicit model following simulation
* Add a section on polytopes for convex optimization? So far, I've seen it used
  for handling saturated control inputs to prioritize tracking some states over
  others using the limited control input.
  * See 971/y2017/control_loops/python/polydrivetrain.py
  * Start with how to turn a set of constraints into a matrix equation of the
    proper form for a polytope, then how to leverage the polytope libs for
    enforcing those constraints.

### Stochastic control theory

* Add an appendix on Ito calculus to explain where the Wiener process comes from?
* Add KF examples for nonlinear drivetrain pose estimation
  * EKF, UKF, and comparison of the two

## Miscellaneous fixes

### State-space controllers

* Modify pareto_boundary.py to find and plot real Pareto boundary for LQR
  instead of using a hand-wavey approximation

### System modeling

* Double check drivetrain J calculation. It doesn't include robot radius, which
  seems suspicious.
