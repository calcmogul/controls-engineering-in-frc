# Practical Guide to State-space Control
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

The scripts can be run like follows:

```
./elevator.py
```

Some Linux platforms use tk as a backend for matplotlib, so that may need to be
installed to see the plots.

## Dependencies

To compile the book, the following are required.

* make (to run the makefile)
* texlive-core (for latexmk)
* texlive-latexextra (for bibtex and makeglossaries)
* texlive-bibtexextra (for additional BibTeX styles and bibliography databases)
* xelatex (for setting and using custom fonts)
* biber (for generating bibliography)
* Python 3.5+
  * frccontrol (to provide FRC wrappers for Python Control)
  * Python Control (to generate plots and state-space results)
    * Installed automatically as a dependency of frccontrol
* Inkscape (to convert SVGs to PDF)

The following are optional because the book can compile without them.

* Optional python packages
  * slycot (to generate pole-zero maps for certain state-space model examples)
  * black (to format Python source code)
* ghostscript (to reduce size of final PDF for publishing)

## Compiling the book

After installing the dependencies, just run `make`. By default, two files will
be generated: state-space-guide.pdf, which has uncompressed images (way higher
than 300dpi); and state-space-guide-ebook.pdf, which is suitable for online
distribution.

The following make targets are supported for compiling the book with various
levels of image compression.

|Command          |File                            |Purpose                  |
|-----------------|--------------------------------|-------------------------|
|`make book-stamp`|`state-space-guide.pdf`         |Quick content development|
|`make ebook`     |`state-space-guide-ebook.pdf`   |Online distribution      |
|`make printer`   |`state-space-guide-printer.pdf` |Standard color printing  |
|`make prepress`  |`state-space-guide-prepress.pdf`|Book publishing          |

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

## Future improvements

### Teach topics more thoroughly

The book is still somewhat dense and fast-paced (it covers three classes of
feedback control, two of which are for graduate students, in one short book).
More examples of concepts should help slow the pace down. I should read through
this book as if I was explaining it to my veteran software students and
anticipate gaps in their understanding.

### Finish incomplete topics

The following state-space implementation examples are in progress:

* Elevator
  * Add u_error state to model
  * Include writing unit tests in Google Test
* Flywheel
  * See elevator items
* Drivetrain
  * See elevator items
  * 971/y2017/control_loops/python/polydrivetrain.py?
  * Implement motion profiles to improve state tracking
* Single-jointed arm
  * See elevator items
  * Implement motion profiles to improve state tracking

The following state-space implementation examples are planned:

* Rotating claw with independent top/bottom
  * See 971/y2014/control_loops/python/claw.py
  * Use as example of coordinate transformations for states?

### Supplementary background

Any other results that are good for background but are unnecessary should be
included in an appendix.

* Add derivations for trapezoidal and S-curve profiles to the derivations
  appendix.
* Import an LQR derivation based on Lagrange multipliers?
* Explain N term for LQR
  * It's a cross term between error and control effort. One application for it
    is making one system behave like some other desired system. This is used on
    the Blackhawk helicopter at NASA Ames when they want to make it fly like
    experimental aircraft (within the limits of the helicopter's actuators, of
    course).
* Add an appendix on Ito calculus to explain where the Wiener process comes from?
* Add equations for Kalman smoother and MMAE.
* Derive the two-sensor and three-sensor problems from first principles. The
  two-sensor problem uses p(x) and p(z_1|x). The three-sensor problem uses p(x),
  p(z_1|x), and p(z_2|x).
* Add example application of Ramsete paper
* Add a section on polytopes for convex optimization? So far, I've seen it used
  for handling saturated control inputs to prioritize tracking some states over
  others using the limited control input.

### Miscellaneous fixes

* Modify pareto_boundary.py to find and plot real Pareto boundary for LQR
  instead of using a hand-wavey approximation
* Fix bugs in dependencies
  * Fix nonsquare system pzmaps in Python Control, then enable non-Slycot pzmaps
    for remaining examples
* Eventually convert the book to ConTeXt

## Licensing

This project, except for the software, is released under the Creative Commons
Attribution-ShareAlike 4.0 International license. The software is released under
the 3-clause BSD license.
