# Practical Guide to State-space Control
## Graduate-level control theory for high schoolers

I originally wrote this as a final project for an undergraduate technical writing class I took at University of California, Santa Cruz in Spring 2017 ([CMPE 185](https://cmpe185-spring17-01.courses.soe.ucsc.edu/)). It is intended as a digest of graduate-level control theory aimed at veteran FIRST Robotics Competition (FRC) students who know algebra and a bit of physics and are comfortable with the concept of a PID controller. As I learned the subject of control theory, I found that it wasn't particularly difficult, but very few resources exist outside of academia for learning it. This document is intended to rectify that situation by providing a lower the barrier to entry to the field.

This document reads a lot like a reference manual on control theory and related tools. It teaches the reader how to start designing and implementing control systems for practical systems with an emphasis on pragmatism rather than theory. While the theory is mathematically elegant at times and helps inform what is going on, one shouldn't lose sight of how it behaves when applied to real systems.

## Download

A PDF version is available at https://file.tavsys.net/tav/control/state-space-guide.pdf.

## Future Improvements

The document is still very high level for the subject it covers as well as very dense and fast-paced (it covers three classes of feedback control, two of which are for graduate students, in one short document). I'd like to expand the introductions for each section and provide more examples like I did for the Kalman filter design to give the reader practice applying the skills discussed.

I also want to convert the control system diagrams to tikz at some point so I can make my own more specialized diagrams.

The "Implementation Steps" section needs subsections to explain how to do each. A small section on kinematics and dynamics, for example, needs to be written.
