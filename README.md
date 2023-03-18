# Controls Engineering in the FIRST Robotics Competition
## Graduate-level control theory for high schoolers

When I was a high school student on FIRST Robotics Competition (FRC) team 3512,
I had to learn control theory from scattered internet sources that either
weren't rigorous enough or assumed too much prior knowledge. After I took
graduate-level control theory courses from University of California, Santa Cruz
for my bachelor's degree, I realized that the concepts weren't difficult when
presented well, but the information wasn't broadly accessible outside academia.

I wanted to fix the information disparity so more people could appreciate the
beauty and elegance I saw in control theory. This book streamlines the learning
process to make that possible.

I wrote the initial draft of this book as a final project for an undergraduate
technical writing class I took at UCSC in Spring 2017
([CMPE 185](https://cmpe185-spring17-01.courses.soe.ucsc.edu/)). It was a
13-page IEEE-formatted paper intended as a reference manual and guide to
state-space control that summarized the three graduate controls classes I had
taken that year. I kept working on it the following year to flesh it out, and it
eventually became long enough to make into a proper book. I've been adding to it
ever since as I learn new things.

I contextualized the material within FRC because it's always been a significant
part of my life, and it's a useful application sandbox. I've since contributed
implementations of many of this book's tools to the FRC standard library
([WPILib](https://github.com/wpilibsuite/allwpilib)) and maintain them.

## Download

A PDF version is available at https://controls-in-frc.link (full link:
https://file.tavsys.net/control/controls-engineering-in-frc.pdf).

## Printed copies

As per [this work's
license](https://github.com/calcmogul/controls-engineering-in-frc/blob/main/LICENSE.CC4),
you may print copies yourself for any purpose under the following terms:

> Attribution — You must give appropriate credit, provide a link to the license,
>               and indicate if changes were made. You may do so in any
>               reasonable manner, but not in any way that suggests the licensor
>               endorses you or your use.

> ShareAlike — If you remix, transform, or build upon the material, you must
>              distribute your contributions under the same license as the
>              original.

Printing a copy of the PDF as-is satisfies the license requirements. You'll want
to use the printer version of the PDF
[here](https://file.tavsys.net/control/controls-engineering-in-frc-printer.pdf)
instead of the one linked above because the latter is compressed for electronic
viewing.

## Running the examples

Example Python scripts can be obtained from frccontrol's Git repository at
https://github.com/calcmogul/frccontrol/tree/main/examples. Furthermore, all
scripts in the [code](code) directory are runnable by the user. They require
Python 3.5+ and frccontrol. frccontrol can be installed with the following
command.

```
pip3 install --user frccontrol
```

Some also use bookutil, a Python package containing common utilities for this
book. That can be installed by running `pip3 install --user -e bookutil` from
the root of this Git repository.

The scripts can be run as follows:

```
./elevator.py
```

Some Linux platforms use tk as a backend for matplotlib, so that may need to be
installed to see the plots.

## Compiling the book

After installing the dependencies, just run `make`. It will produce a PDF named
controls-engineering-in-frc.pdf.

### Dependencies

To compile the book, the following packages are required.

#### Arch Linux

These can be installed via `make setup_archlinux`.

* pacman packages (via `sudo pacman -Sy`)
  * base-devel (for `make` to run the makefile)
  * biber (for generating bibliography)
  * clang (for clang-format to format C++ code snippets)
  * cmake (for building C++ plot generators)
  * imagemagick (to compress JPEGs)
  * inkscape (to convert SVGs to PDFs)
  * perl-clone (for Clone.pm needed by biber)
  * python >= 3.6 (for generating plots)
  * python-black (to format Python source code)
  * python-pip (for installing required Python packages)
  * python-pylint (for Python linting)
  * python-requests (for .tex HTTP link checker)
  * python-wheel (for "setup.py bdist_wheel")
  * texlive-bibtexextra (for additional BibTeX styles and bibliography databases)
  * texlive-core (for latexmk and xelatex)
  * texlive-latexextra (for bibtex and makeglossaries)

#### Ubuntu

These can be installed via `make setup_ubuntu`.

* apt packages (via `sudo apt install`)
  * biber (for generating bibliography)
  * build-essential (for `make` to run the makefile)
  * cm-super (for type1ec.sty)
  * clang-format (to format C++ code snippets)
  * cmake (for building C++ plot generators)
  * dvipng (to convert DVIs to PNGs)
  * imagemagick (to compress JPEGs)
  * inkscape (to convert SVGs to PDFs)
  * latexmk
  * python3 >= 3.6 (for generating plots)
  * python3-pip (for installing required Python packages)
  * python3-requests (for .tex HTTP link checker)
  * python3-setuptools (for dependencies of setup.py)
  * python3-wheel (for "setup.py bdist_wheel")
  * texlive-base (for latexmk)
  * texlive-bibtex-extra (for additional BibTeX styles and bibliography databases)
  * texlive-generic-extra (for miscellaneous LaTeX .sty files)
  * texlive-latex-extra (for bibtex and makeglossaries)
  * texlive-xetex (for xelatex)
* Python packages (via `pip3 install --user`)
  * black (to format Python source code)
  * pylint (for Python linting)

#### macOS

These can be installed via `make setup_macos`.

* brew packages (via `brew install`)
  * basictex (for xelatex)
  * clang-format (to format C++ code snippets)
  * cmake (for building C++ plot generators)
  * imagemagick (to compress JPEGs)
  * inkscape (to convert SVGs to PDFs)
  * python@3.10 (for generating plots)
* Python packages (via `pip3 install --user`)
  * black (to format Python source code)
  * pylint (for Python linting)
* tlmgr packages (via `sudo tlmgr install`)
  * biber (for generating bibliography)
  * biblatex (for generating bibliography)
  * cm-super (for type1ec.sty)
  * csquotes (used by textcomp package)
  * datatool (used by xfor package)
  * enumitem (customize lists)
  * footmisc (used by gensymb)
  * gensymb
  * glossaries (for makeglossaries command)
  * glossaries-english (english language module for glossaries package)
  * imakeidx (used by listings package)
  * latexmk
  * mdframed (for creating the theorem, definition, exercise, and corollary
    boxes)
  * mfirstuc (used by glossaries package)
  * needspace (used by zref package)
  * placeins (used by subcaption package)
  * titlesec (for titletoc package)
  * tracklang (used by glossaries package)
  * type1cm (for type1cm.sty)
  * was (for gensymb package)
  * xfor (used by textcase package)
  * zref (used by mdframed package)

#### Python packages

These packages are installed via pip3 (e.g., `pip3 install --user frccontrol`).

* frccontrol (to provide FRC wrappers for scipy and generate plots and
  state-space results)

The book's build process automatically sets these up in a venv so they don't
have to be installed manually. Modifications to the Python package folders in
`build` will be reflected in any scripts which use the venv.

### Style guide

All LaTeX labels, including bibliography entries, should use underscores to
represent spaces between words instead of hyphens. Hyphens should only be used
where the character being written is already a hyphen. `.tex` file names should
use hyphens and `.py` file names should use underscores.

When including linebreaks in equations, insert the linebreak and a `\qquad`
before the next operator.

Glossary entries in [glossary-entries.tex](glossary-entries.tex) should be
lexographically sorted by entry key. The entry key should be the same as the
name. The words in the name should be lowercase, and the description should
start with a capital letter and end with a period. The first sentence should be
a fragment, but sentences after that, if applicable, should be complete.

Bibliography entries in
[controls-engineering-in-frc.bib](controls-engineering-in-frc.bib) should be
sorted lexographically by label. Links to online videos should use the `@misc`
tag. Links to online static resources like PDFs should use the `@online` tag.

Book content should answer the question _why_ something works the way it does
and how and when to use it to solve problems.

Adhere to [SI unit rules and style
conventions](https://physics.nist.gov/cuu/Units/checklist.html).

## Future improvements

See [TODO.md](TODO.md).

## Licensing

This project, except for the software, is released under the Creative Commons
Attribution-ShareAlike 4.0 International license. The software is released under
the 3-clause BSD license.
