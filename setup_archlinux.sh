#!/bin/bash

# pacman packages
#   * base-devel (for `make` to run the makefile)
#   * biber (for generating bibliography)
#   * clang (for clang-format to format C++ code snippets)
#   * cmake (for building C++ plot generators)
#   * imagemagick (to compress JPEGs)
#   * inkscape (to convert SVGs to PDFs)
#   * perl-clone (for Clone.pm needed by biber)
#   * python >= 3.6 (for generating plots)
#   * python-black (to format Python source code)
#   * python-pip (for installing required Python packages)
#   * python-pylint (for Python linting)
#   * python-requests (for .tex HTTP link checker)
#   * texlive-bibtexextra (for LaTeX package biblatex)
#   * texlive-bin (for xelatex)
#   * texlive-binextra (for matplotlib usetex dependencies)
#   * texlive-fontsrecommended (for matplotlib usetex dependencies and fonts
#                               needed by LaTeX package amsmath)
#   * texlive-latex (for essential LaTeX packages)
#   * texlive-latexextra (for LaTeX package csquotes)
#   * texlive-latexrecommended (for LaTeX packages amsmath, graphicx, etc.)
#   * texlive-plaingeneric (for plain (La)TeX packages)
#   * texlive-xetex (for XeTeX)
sudo pacman -Sy --needed --noconfirm \
  base-devel \
  biber \
  clang \
  cmake \
  imagemagick \
  inkscape \
  perl-clone \
  python \
  python-black \
  python-pip \
  python-pylint \
  python-qrcode \
  python-requests \
  texlive-bibtexextra \
  texlive-bin \
  texlive-binextra \
  texlive-fontsrecommended \
  texlive-latex \
  texlive-latexextra \
  texlive-latexrecommended \
  texlive-plaingeneric \
  texlive-xetex

# autoflake isn't in [extra] and we can't use an AUR helper
pip3 install --user --break-system-packages autoflake
