# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join("..", "..", "python")))


# -- Project information -----------------------------------------------------

project = 'uPSP'
copyright = '2021, uPSP Developers'
author = 'uPSP Developers'


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.autosectionlabel",
    "sphinx.ext.intersphinx",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
    "sphinxcontrib.bibtex",
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
# html_theme = "sphinx_rtd_theme"
html_theme_options = "alabaster"
html_theme_options = {
    "fixed_sidebar": True,
    "page_width": "1200px",
    "globaltoc_maxdepth": 2,
}

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']


# autodoc config
autodoc_default_options = {
    "undoc-members": True,
}
autodoc_mock_imports = ["cv2", "upsp.raycast"]

# just show the method/function name in signatures, not the full path
add_module_names = False

numfig = True

autosectionlabel_prefix_document = True

latex_engine = "pdflatex"
latex_elements = {
    "fontpkg": "",
}

rst_prolog = """
.. role:: python(code)
    :language: python
    :class: highlight
"""

bibtex_bibfiles = ['refs.bib']

napoleon_preprocess_types = True
napoleon_type_aliases = {
    "np.ndarray": ":class:`numpy.ndarray`",
    "ndarray": ":class:`numpy.ndarray`",
    "array_like": ":term:`array_like`",
    "array-like": ":term:`array-like <array_like>`",
    "path-like": ":term:`path-like <path-like object>`",
}

intersphinx_mapping = {
    "python": ("https://docs.python.org/3", None),
    "numpy": ("https://numpy.org/doc/stable/", None),
    "scipy": ("https://docs.scipy.org/doc/scipy/", None),
    "pandas": ("https://pandas.pydata.org/docs", None),
    "matplotlib": ("https://matplotlib.org/stable/", None),
}

# nitpicky = True
