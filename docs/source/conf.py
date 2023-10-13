# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information


import os
import sys
sys.path.insert(0, os.path.abspath('.'))
sys.path.insert(0, os.path.abspath('../'))
sys.path.insert(0, os.path.abspath('../../catkin_ws/src/guerleboat/scripts'))
sys.path.insert(0, os.path.abspath('../../catkin_ws/src/guerledocking/scripts'))


project = 'Guerledocking'
copyright = '2023, Kévin REN, Théo MASSA, Guillaume GARDE, Hugo HOFMANN'
author = 'Kévin REN, Théo MASSA, Guillaume GARDE, Hugo HOFMANN'
release = '1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.viewcode',
    'sphinx_copybutton',
    "myst_parser",
    'sphinx.ext.napoleon'
]


autodoc_mock_imports = ['rospy',
                        'geometry_msgs',
                        'std_msgs',
                        'nav_msgs',
                        'tf2_ros',
                        'turtlebot3_msgs',
                        'cv_bridge',
                        'sensor_msgs',
                        'roblib',
                        'sbg_driver'
                        ]

templates_path = ['_templates']
exclude_patterns = []

source_suffix = ['.rst', '.md']


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "sphinx_rtd_theme"#'alabaster'
html_static_path = ['_static']
