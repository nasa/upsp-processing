"""Context management for uPSP data processing

Single module for encapsulating the layout of the
uPSP data processing hierarchy, and providing convenient
routines for retrieving resource paths from the output
folder tree.
"""
import os
import posixpath

from .io import json_load_or_die


class Pipeline:
    """
    uPSP Data Pipeline context management

    The Pipeline class facilitates reading and writing data
    into the processed data file hierarchy on disk. Given
    the top-level "root directory," the class provides methods
    to resolve fully-qualified paths to files in the tree on disk.
    """
    def __init__(self, root_dir: str, ctx_filename=None):
        self.root_dir = root_dir
        self.output_dir = posixpath.join(self.root_dir, "05_products", "00_data")
        self.ctx_filename = ctx_filename or posixpath.join(self.root_dir, "context.json")
        self.ctx = json_load_or_die(self.ctx_filename)

    @property
    def datapoints(self):
        return [self.datapoint(name) for name in self.ctx["datapoints"].keys()]

    def datapoint(self, name: str):
        return Datapoint(self, name)


class Datapoint:
    """
    uPSP Data Pipeline per-datapoint context management

    The Datapoint class facilitates reading and writing data
    into the processed data file hierarchy on disk on a per-datapoint
    basis. Given a Pipeline object, the class extends the Pipeline
    object functionality to provide the ability to resolve paths
    to resources for a single datapoint in the folder tree, as
    well as to easily identify source input files.
    """
    def __init__(self, pipeline: Pipeline, name: str):
        self.pipeline = pipeline
        self.name = str(name)

    @property
    def inputs(self):
        return self.pipeline.ctx["datapoints"][self.name]

    def output_path(self, pipeline_process_name: str, *args):
        return posixpath.join(
            self.pipeline.output_dir,
            pipeline_process_name,
            self.name,
            *args
        )
