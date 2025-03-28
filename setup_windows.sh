#!/bin/bash

choco install \
  imagemagick \
  inkscape
choco install texlive --version=2025.20250318.0 --params="'/collections:basic /extraPackages:amscls,amsmath,babel-english,biber,biblatex,bookmark,booktabs,caption,cm-super,colortbl,csquotes,datatool,enumitem,fancyhdr,float,fontspec,footmisc,gensymb,geometry,glossaries,glossaries-english,hyperref,imakeidx,infwarerr,kvsetkeys,latex,latexmk,listings,ltxcmds,mathtools,mdframed,microtype,needspace,parskip,pgf,placeins,titlesec,tools,tracklang,type1cm,was,xetex,xfor,zref'"

# Python packages
#  * black (to format Python source code)
#  * pillow (required by qrcode, but not preinstalled)
#  * pylint (for Python linting)
#  * qrcode (for QR codes)
#  * requests (for .tex HTTP link checker)
pip install --user autoflake black==25.1.0 pillow pylint qrcode requests wheel
