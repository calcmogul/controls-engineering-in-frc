#!/bin/bash

choco install \
  imagemagick \
  inkscape
choco install texlive --params="'/collections:basic /extraPackages:biber,biblatex,cm-super,csquotes,datatool,enumitem,footmisc,glossaries,glossaries-english,imakeidx,latex,latexextra,latexmk,latexrecommended,mdframed,mfirstuc,needspace,placeins,titlesec,tracklang,type1cm,was,xetex,xfor,zref'"
setx path "%PATH%;C:\texlive\2022\bin\win32"

# Python packages
#  * black (to format Python source code)
#  * pylint (for Python linting)
pip3 install --user autoflake black==24.2.0 qrcode requests wheel
