import collections.abc
import datetime
import glob
import logging
import os
import re
import shutil
import string
import sys
import textwrap
import numpy as np

import upsp
from . import io

log = logging.getLogger(__name__)


def _datapoint_processing_config(cfg: dict, step_name: str, run_number: str):
    return cfg["processing"][run_number][step_name]


def _camera_filenames(cfg, run_number, exts=[".cine", ".dyn", ".mraw"]):
    # Finds individual camera video files for this run number in its "camera_video_dir".
    # Valid camera video file extensions are supplied via the "exts" parameter.
    # Assumes camera files are named as "<datapoint><camera number>.<extension>"
    camera_dir = cfg["datapoints"][run_number]["camera_video_dir"]
    candidate_filenames = [
        f
        for f in glob.glob(os.path.join(camera_dir, "%s*" % run_number))
        if os.path.splitext(f)[1] in exts
    ]
    # sort all files into a list per camera number
    camera_candidate_filenames = {}
    for f in candidate_filenames:
        n = _camera_number_from_filename(f)
        if n not in camera_candidate_filenames:
            camera_candidate_filenames[n] = []
        camera_candidate_filenames[n].append(f)
    # then, return first file from list for each camera.
    # warn if there's more than one element (ambiguous which one to use)
    nlst = sorted(camera_candidate_filenames.keys())
    if len(nlst) != max(nlst):
        log.warning("Missing one or more camera files (found %s)", nlst)
    camera_filenames = [""] * len(nlst)
    for n, lst in camera_candidate_filenames.items():
        if len(lst) > 1:
            log.warning(
                "More than one file found for %s, camera %02d: %s", run_number, n, lst
            )
        camera_filenames[n - 1] = lst[0]
    return camera_filenames


def _camera_number_from_filename(filename):
    # <datapoint><camera number>.<extension>
    return int(os.path.splitext(os.path.basename(filename))[0][6:])


def _camera_name(filename):
    return "cam%02d" % _camera_number_from_filename(filename)


_ADD_FIELD_EXE = shutil.which("add_field")

_DEFAULT_TASK_EXE_NAME = "task.sh"

_NAS_NCPUS_BY_MODEL = {
    "cas_ait": 40,
    "sky_ele": 40,
    "bro_ele": 28,
    "bro": 28,
    "has": 24,
    "ivy": 20,
    "san": 16,
}


class Error(Exception):
    pass


def ensure_unique(values: list[str], prefixes=None):
    # Ensure a list of strings is unique. If not, add prefixes to non-unique elements.
    # If prefixes is not specified, the list index (0, 1, 2, ...) is used.
    prefixes = list(range(len(values))) if prefixes is None else prefixes
    prefixes = [str(s) for s in prefixes]

    add_prefix = np.array([False, False, False, False], dtype=bool)
    uniq_values, inv_idxs = np.unique(values, return_inverse=True)
    for ii in range(len(uniq_values)):
        duplicate_check = inv_idxs == ii
        if np.count_nonzero(duplicate_check) > 1:
            add_prefix[duplicate_check] = True
    new_values = np.where(
        add_prefix, list(map(''.join, zip(prefixes, values))), values
    )
    return new_values


def copy_files_ensure_unique_names(src_filenames, dst_dir, src_prefixes=None):
    src_basenames = [os.path.basename(fn) for fn in src_filenames]
    dst_basenames = ensure_unique(src_basenames, prefixes=src_prefixes)
    for src_fn, dst_bn in zip(src_filenames, dst_basenames):
        dst_fn = os.path.join(dst_dir, dst_bn)
        shutil.copy(src_fn, dst_fn)
        print("Copied:", src_fn, '->', dst_fn)

# Create a processing tree for uPSP raw data.
#
# The processing tree is a hierarchy to contain
# - Launcher scripts to run portions (or all of) the processing steps
# - Logs and diagnostics from processing steps
# - Output data artifacts from processing steps
#
# The create() routine is not itself intended to run
# any "heavyweight" processing, but rather to create the *environment*
# for performing the processing.
#
def create(
    output_dir: os.PathLike,
    data_config_filename: os.PathLike,
    user_config_filename: os.PathLike,
    proc_config_filename: os.PathLike,
    plot_config_filename: os.PathLike,
):
    dat = io.json_load_or_die(data_config_filename)
    usr = io.json_load_or_die(user_config_filename)
    swr = io.json_load_or_die(proc_config_filename)
    plt = io.json_load_or_die(plot_config_filename)

    proc = _resolve_parameter_overlays(swr["processing"], dat["datapoints"])
    nas = _resolve_nas_config(usr["nas"])

    cfg = {
        "datapoints": dat["datapoints"],
        "nas": nas,
        "root": output_dir,
        "processing": proc,
        "plotting": plt["plotting"],
        "__meta__": {
            "datapoints": dat["__meta__"],
            "nas": usr["__meta__"],
            "processing": swr["__meta__"],
            "plotting": plt["__meta__"],
            "__date__": datetime.date.today().strftime("%b-%d-%Y"),
        },
    }

    dirname = _create(cfg)

    # Write the final cfg state out to an index file
    ctx_filename = os.path.join(dirname, "context.json")
    io.json_write_or_die(cfg, ctx_filename, indent=2)

    # Write the origin JSON files to the config directory
    # ... if their basenames aren't unique, MAKE them unique
    # (to avoid overwriting eachother. It's possible they have
    # the same basename but are located in different folders)
    _cfgs = {
        "data-": data_config_filename, "user-": user_config_filename,
        "proc-": proc_config_filename, "plot-": plot_config_filename
    }
    copy_files_ensure_unique_names(
        list(_cfgs.values()),
        _configuration_path(cfg),
        src_prefixes=list(_cfgs.keys())
    )


def _resolve_nas_config(nas: dict):
    # Resolve NAS launch parameters for each pipeline step
    #
    # - Output is a dictionary containing launch parameters for each pipeline step
    # - Input is the dictionary containing config JSON
    #
    # Applies defaults to each step, then overlays step-specific values.
    out = {}
    for name in [k for k in nas.keys() if k != "__defaults__"]:
        o = {k: v for k, v in nas["__defaults__"].items()}
        o.update(nas[name])
        out[name] = o
    return out


def _resolve_parameter_overlays(processing: dict, datapoints: dict):
    # Resolve processing parameters for each datapoint
    #
    # - Output is a dictionary containing the parameter values for each
    #   datapoint.
    #
    # - Inputs are the processing JSON and the datapoint index.
    #
    # Example showing how to override one parameter for pipeline 'appA' based on
    # whether the datapoint grid file matches a given naming convention.
    #
    # - Processing JSON:
    #
    #     ```json
    #     {
    #       "defaults": {"appA": {"pa1": 1, "pa2": 2}, "appB": {"pb1": 3, "pb2": 4}},
    #       "config1": {"appA": {"pa1": 7}},
    #       "__overlays__": [
    #         ("defaults", {".*": ".*"}),
    #         ("config1", {"grid_file": ".*config1.grid"})
    #       ]
    #     }
    #     ```
    #
    # - Datapoint index JSON
    #
    #     ```json
    #     {
    #       "000000": {"grid_file": "/path/to/config0.grid", ...},
    #       "111111": {"grid_file": "/path/to/config1.grid", ...}
    #     }
    #     ```
    #
    # - Output context JSON:
    #
    #     ```json
    #     {
    #      "000000": {"appA": {"pa1": 1, "pa2": 2}, "appB": {"pb1": 3, "pb2": 4}},
    #      "111111": {"appA": {"pa1": 7, "pa2": 2}, "appB": {"pb1": 3, "pb2": 4}}
    #     }
    #     ```
    #
    def _match_all_patterns(patterns, dp_name, dp):
        # Return True if ALL patterns match
        # A pattern matches if:
        # - its key matches at least one input name, AND
        # - for each key match, its value matches the input value
        def _match_pattern(d, pkey, pval):
            pattern_key_matched_once = False
            for dkey, dval in d.items():
                if re.fullmatch(pkey, dkey):
                    pattern_key_matched_once = True
                    if re.fullmatch(pval, dval):
                        return True
            if not pattern_key_matched_once:
                log.warning(
                    "%s: Overlay pattern '%s' does not match any input names (%s)",
                    dp_name,
                    pkey,
                    list(d.keys()),
                )
            return False

        matches = [_match_pattern(dp, *item) for item in patterns.items()]
        return all(matches)

    def _update(d, u):
        for k, v in u.items():
            if isinstance(v, collections.abc.Mapping):
                d[k] = _update(d.get(k, {}), v)
            else:
                d[k] = v
        return d

    def _validate_overlay_syntax(overlays):
        def _validate_re(s):
            try:
                re.compile(s)
            except re.error as e:
                err = "In __overlays__: invalid regular expression '%s' (%s)" % (
                    s,
                    str(e),
                )
                raise Error(err)

        for ov in overlays:
            for pkey, pval in ov[1].items():
                _validate_re(pkey)
                _validate_re(pval)

    param_sets = {k: v for k, v in processing.items() if k != "__overlays__"}
    overlays = processing.get("__overlays__", {})
    proc = {k: {} for k in datapoints.keys()}
    _validate_overlay_syntax(overlays)
    for ov in overlays:
        overlay_matched_once = False
        for dp_name, dp in datapoints.items():
            name, patterns = ov
            if _match_all_patterns(patterns, dp_name, dp):
                overlay_matched_once = True
                proc[dp_name] = _update(proc[dp_name], param_sets[name])
        if not overlay_matched_once:
            log.warning("Overlay '%s' matched no datapoints, will be a no-op", ov)
    return proc


def _launcher_env_sh(cfg: dict):
    """Returns string containing sh environment setup commands

    - Load any required system libraries that aren't available
      by default... for example, system-provided MPI libraries.
    
    - Prefix the PATH with the directory of our current python
      interpreter. Ensures any scripts keying off "/usr/bin/env python"
      resolve the correct interpreter at runtime.
    """
    env_sh_lines = [
        "source /usr/local/lib/global.profile",
        "module purge",
        "module load mpi-hpe/mpt.2.25",
        "export PATH=%s:$PATH" % (os.path.dirname(shutil.which("python")))
    ]
    return "\n".join(env_sh_lines)


def _create_qsub_step_launcher(cfg: dict):
    filename = _launchers_path(cfg, "qsub-step")
    env_sh = _launcher_env_sh(cfg)
    exe_sh = textwrap.dedent(
        f"""
        SCRIPT_DIR=$( cd -- "$( dirname -- "${{BASH_SOURCE[0]}}" )" &> /dev/null && pwd )
        STEP_LAUNCHER_FILENAME=$(realpath $1)
        STEP_LAUNCHER_BASENAME=$(basename $STEP_LAUNCHER_FILENAME)
        STEP_NAME="$(echo ${{STEP_LAUNCHER_BASENAME}} | cut -d+ -f2)"
        D=$SCRIPT_DIR/../04_processing/01_exec/$STEP_NAME
        ALL_DATAPOINTS=($(ls -1 $D))
        INP_DATAPOINTS=("${{@:2}}")
        SEL_DATAPOINTS=()
        if (( ${{#INP_DATAPOINTS[@]}} )); then
            SEL_DATAPOINTS+=( "${{INP_DATAPOINTS[@]}}" )
        else
            SEL_DATAPOINTS+=( "${{ALL_DATAPOINTS[@]}}" )
        fi
        readarray -t UPSP_QSUB_ARGS_OUT < \
            <({shutil.which("upsp-qsub-args")} "$SCRIPT_DIR/.." $STEP_NAME ${{SEL_DATAPOINTS[@]}})
        NLINES="${{#UPSP_QSUB_ARGS_OUT[@]}}"
        IDX=0
        while [ $IDX -lt $NLINES ]; do
            THIS_QSUB_ARGS=${{UPSP_QSUB_ARGS_OUT[$IDX]}}
            IDX=$(( $IDX + 1 ))
            THIS_DATAPOINTS=${{UPSP_QSUB_ARGS_OUT[$IDX]}}
            IDX=$(( $IDX + 1 ))
            CMD="$STEP_LAUNCHER_FILENAME $THIS_DATAPOINTS"
            EX="qsub $THIS_QSUB_ARGS -- $CMD"
            echo "$EX"
            $EX
        done
        """
    )
    filename = _launchers_path(cfg, "qsub-step")
    _create_launcher(filename, pbs_sh="", env_sh=env_sh, exe_sh=exe_sh)


def _create_step_launcher(cfg: dict, step_name: str, exe_name=""):
    if exe_name:
        exe_alias = os.path.splitext(exe_name)[0].replace("run-", "")
        filename = _launchers_path(cfg, "step+%s+%s" % (step_name, exe_alias))
    else:
        exe_name = "task.sh"
        filename = _launchers_path(cfg, "step+%s" % step_name)
    launcher_type = cfg["nas"][step_name]["launcher"]
    TEMPLATE_FILE = os.path.join(
        os.path.dirname(__file__),
        "templates",
        "run-step-%s.sh.template" % (launcher_type,),
    )
    with open(TEMPLATE_FILE, "r") as fp:
        ts = fp.read()
    outp = string.Template(ts).substitute(
        __pipeline_root_dir__=_root_path(cfg),
        __pipeline_step_name__=step_name,
        __pipeline_step_exe_name__=exe_name,
        __pbs_sh__="",
    )
    with open(filename, "w") as fp:
        fp.write(outp)
    os.chmod(filename, io._CHMOD_URWXGR_XO__)


def _create_launcher(filename, pbs_sh="", env_sh="", exe_sh=""):
    TEMPLATE_FILE = os.path.join(
        os.path.dirname(__file__), "templates", "launcher.sh.template"
    )
    with open(TEMPLATE_FILE, "r") as fp:
        ts = fp.read()
    outp = string.Template(ts).substitute(
        __pbs_sh__=pbs_sh, __env_sh__=env_sh, __exe_sh__=exe_sh
    )
    with open(filename, "w") as fp:
        fp.write(outp)
    os.chmod(filename, io._CHMOD_URWXGR_XO__)


def _create_plotting_config_file(cfg: dict, name: str):
    _create_dir_and_log(_app_cache_path(cfg, name))
    filename = _app_cache_path(cfg, name, name + ".json")
    subcfg = cfg["plotting"][name]
    io.json_write_or_die(subcfg, filename, indent=2)
    return filename


def _create_add_field_file(cfg: dict, step_name: str, run_number: str, exe_name: str):
    # post-processing step using 'add_field' executable, to add flat file data
    # into the partially-complete H5 file. Should run AFTER psp_process finishes.
    # This creates a shell script that wraps said call to 'add_field' and writes
    # useful debug info.
    env_sh = _launcher_env_sh(cfg)

    TEMPLATE_FILE = os.path.join(
        os.path.dirname(__file__), "templates", "add-field.sh.template"
    )
    with open(TEMPLATE_FILE, "r") as fp:
        ts = fp.read()
    exe_sh = string.Template(ts).substitute(
        add_field_exe=_ADD_FIELD_EXE,
        datapoint_logs_dir=_app_logs_path(cfg, step_name, run_number),
        datapoint_output_dir=_app_products_path(cfg, step_name, run_number),
    )

    add_field_filename = _app_exec_path(cfg, step_name, run_number, exe_name)
    _create_launcher(add_field_filename, env_sh=env_sh, exe_sh=exe_sh)
    return add_field_filename


def _create_pbs_file(cfg: dict, step_name: str, run_number: str, exe_name: str):
    """Generate the job script for a single datapoint
    Also create directories needed by the job at runtime
    """
    this_run = cfg["datapoints"][run_number]
    pcfg = _datapoint_processing_config(cfg, step_name, run_number)
    nas = cfg["nas"][step_name]
    ncpus = _NAS_NCPUS_BY_MODEL[nas["node_model"]]
    run_name = _run_name(cfg, run_number)
    exe_prefix = os.path.splitext(exe_name)[0]
    stdout = _app_logs_path(cfg, step_name, run_number, exe_prefix + ".stdout")
    stderr = _app_logs_path(cfg, step_name, run_number, exe_prefix + ".stderr")
    pbs_sh_rows = [
        "#PBS -N %s" % (run_name),
        "#PBS -q %s" % (nas["queue"]),
        "#PBS -W group_list=%s" % (nas["charge_group"]),
        "#PBS -l select=%s:model=%s" % (nas["number_nodes"], nas["node_model"]),
        "#PBS -l walltime=%s" % (nas["wall_time"]),
        "#PBS -o %s" % stdout,
        "#PBS -e %s" % stderr,
        "export MPI_DSM_DISTRIBUTE=off",
        "export OMP_STACKSIZE=250M",
        "export OMP_NUM_THREADS=%s" % (ncpus - 4),
    ]
    pbs_sh = "\n".join(pbs_sh_rows)

    env_sh = _launcher_env_sh(cfg)

    h5_filename = _app_products_path(
        cfg, step_name, run_number, "pressure_transpose.h5"
    )
    log_filename = _app_logs_path(cfg, step_name, run_number, "psp-process.out")

    exe_rows = [
        "# By default, core dumps are written out to the current working",
        "# directory of the process. We want these core dumps to be written",
        "# to the process logs directory.",
        "cd %s" % _app_logs_path(cfg, step_name, run_number),
        "mpiexec %s \\" % (shutil.which("psp_process"),),
        "  -cutoff_x_max=%s \\" % (pcfg["cutoff_x_max"]),
        "  -input_file=%s \\" % (_inp_filename(cfg, step_name, run_number)),
        "  -h5_out=%s \\" % (h5_filename),
        "  -paint_cal=%s \\" % (this_run["paint_calibration_file"]),
        "  -steady_p3d=%s \\" % (this_run["steady_psp_file"]),
        "  -frames=%s \\" % (pcfg["number_frames"]),
        "  -code_version=%s \\" % (upsp.__version__),
        "  > %s 2>&1\n" % (log_filename),
    ]
    exe_sh = "\n".join(exe_rows)

    filename = _app_exec_path(cfg, step_name, run_number, exe_name)
    _create_launcher(filename, pbs_sh=pbs_sh, env_sh=env_sh, exe_sh=exe_sh)
    _create_dir_and_log(os.path.dirname(stdout))
    _create_dir_and_log(os.path.dirname(stderr))
    _create_dir_and_log(os.path.dirname(h5_filename))
    return filename


def _create_input_file(
    cfg: dict, step_name: str, run_number: str, external_calibration_info: dict
):
    """Generate input deck file for run number"""
    this_run = cfg["datapoints"][run_number]
    pcfg = _datapoint_processing_config(cfg, step_name, run_number)
    camera_files = _camera_filenames(cfg, run_number)
    wind_tunnel_str = "ames_unitary"
    frame_rate = 10000
    fstop = 2.8
    input_rows = [
        "%%Version %s" % upsp.__version__,
        "%%Date_Created: %s" % cfg["__meta__"]["__date__"],
        "",
        "@general",
        "\ttest = %s" % _test_name(cfg),
        "\trun = %s" % run_number[0:4],
        "\tsequence = %s" % run_number[4:6],
        "\ttunnel = %s" % wind_tunnel_str,
        "\tframerate = %d" % frame_rate,
        "\tfstop = %4.1f" % fstop,
        "@all",
        "\tgrid = %s" % this_run["grid_file"],
        "\tsds = %s" % this_run["wtd_file"],
        "\ttargets = %s" % this_run["targets_file"],
        "\tnormals = %s" % this_run.get("normals_file", ""),
    ]

    for idx, filename in enumerate(camera_files):
        calibration_filename = external_calibration_info["calibration_filename"][idx]
        input_rows += [
            "@camera",
            "\tnumber = %d" % (idx + 1,),
            "\tcine = %s" % filename,
            "\tcalibration = %s" % calibration_filename,
        ]

    input_rows += [
        "@options",
        "\ttarget_patcher = %s" % (pcfg["target_patcher"]),
        "\tregistration = %s" % (pcfg["registration"]),
        "\tfilter = %s" % (pcfg["filter"]),
        "\tfilter_size = %s" % (pcfg["filter_size"]),
        "\toblique_angle = %s" % (pcfg["oblique_angle"]),
        "\tnumber_frames = -1",  # number_frames will be set in PBS file cmdline args
    ]

    input_rows += [
        "@output",
        "\tdir = %s" % (_app_products_path(cfg, step_name, run_number)),
        "\tname = %s" % (run_number),
    ]

    input_str = "\n".join(input_rows)
    filename = _inp_filename(cfg, step_name, run_number)
    _create_dir_and_log(os.path.dirname(filename))
    with open(filename, "w") as fid:
        fid.write(input_str)
    os.chmod(filename, io._CHMOD_URW_GR__O__)


def _test_name(cfg: dict):
    return cfg["__meta__"]["datapoints"]["test_name"]


def _configuration_name(cfg: dict):
    # The overall processing configuration name is a combination of:
    # - The alias for configurable input files for each datapoint
    #   (e.g., the steady-state pressure input files, or the model grid files)
    #   (input files that are NOT configurable are, e.g., raw cine files)
    # - The alias for the uPSP processing software parameter values
    alias_input_files = cfg["__meta__"]["datapoints"]["config_name"]
    alias_processing_params = cfg["__meta__"]["processing"]["name"]
    toks = [alias_input_files, alias_processing_params]
    filtered_toks = [t for t in toks if t != "default"]
    if not filtered_toks:
        filtered_toks = ["default"]
    return "+".join(filtered_toks)


def _version_configuration_name(cfg: dict):
    # Combine software version and configuration name into a single string.
    return "+".join([upsp.__version__, _configuration_name(cfg)])


def _root_path(cfg: dict, *args):
    return os.path.join(
        cfg["root"], _test_name(cfg), _version_configuration_name(cfg), *args
    )


def _configuration_path(cfg: dict, *args):
    return _root_path(cfg, "02_configuration", *args)


def _launchers_path(cfg: dict, *args):
    return _root_path(cfg, "03_launchers", *args)


def _processing_path(cfg: dict, *args):
    return _root_path(cfg, "04_processing", *args)


def _products_path(cfg: dict, *args):
    return _root_path(cfg, "05_products", *args)


def _processing_exec_path(cfg: dict, *args):
    return _processing_path(cfg, "01_exec", *args)


def _processing_logs_path(cfg: dict, *args):
    return _processing_path(cfg, "02_logs", *args)


def _app_exec_path(cfg: dict, app_name: str, *args):
    return _processing_exec_path(cfg, app_name, *args)


def _app_logs_path(cfg: dict, app_name: str, *args):
    return _processing_logs_path(cfg, app_name, *args)


def _app_products_path(cfg: dict, app_name: str, *args):
    return _products_path(cfg, "00_data", app_name, *args)


def _app_cache_path(cfg: dict, app_name: str, *args):
    return _products_path(cfg, "00_data", "cache", app_name, *args)


def _inp_filename(cfg: dict, step_name: str, run_number: str):
    return _app_exec_path(cfg, step_name, run_number, "psp-process.inp")


def _create_dir_and_log(path: str):
    if not os.path.exists(path):
        os.makedirs(path)
        log.info("Created %s", path)


def _create_root_dir(cfg: dict):
    root_dir = _root_path(cfg)
    if os.path.exists(root_dir):
        io.user_confirm_or_exit("'%s' exists, force-overwrite?" % root_dir)
    try:
        _create_dir_and_log(root_dir)
    except PermissionError as err:
        log.error(err)
        log.error("Could not create output directory.")
        log.error("Check user 'data_processing_dir' parameter.")
        sys.exit(1)


def _run_name(cfg: dict, run_number: str):
    return "-".join([run_number, _version_configuration_name(cfg)])


def _create_task_render_images(cfg: dict, step_name: str, run_number: str):
    # - For each datapoint, step to render views of model using Tecplot
    #   (uses NAS tecplot license)
    cfg["plotting"]["render-images"]["runtime"] = {
        "table_generator_exe": "xyz_scalar_to_tbl",
        "images_dir": _app_products_path(cfg, "images"),
        "cache_dir": _app_cache_path(cfg, "tecplot"),
        "root_dir": _root_path(cfg),
    }
    # TODO This file will be overwritten every time this is run per-datapoint.
    #      Inefficient but not a huge deal.
    cfg_filename = _create_plotting_config_file(cfg, "render-images")
    exe_sh = "%s render-images %s %s" % (
        shutil.which("upsp-plotting"), cfg_filename, run_number
    )
    env_sh = _launcher_env_sh(cfg)
    exe_filename = _app_exec_path(cfg, step_name, run_number, _DEFAULT_TASK_EXE_NAME)
    _create_dir_and_log(_app_logs_path(cfg, step_name, run_number))
    _create_dir_and_log(_app_exec_path(cfg, step_name, run_number))
    _create_launcher(exe_filename, exe_sh=exe_sh, env_sh=env_sh)
    return None, [_DEFAULT_TASK_EXE_NAME]


def _create_task_extract_first_frame(cfg: dict, step_name: str, run_number: str):
    # - For each datapoint, step to dump first frame from each camera

    img_ext = "png"
    img_frame_number = 1

    def _first_frames_info(cfg: dict, run_number: str):
        """Paths to data products - first frame from each camera video file.

        Intermediate data product output by "extract-first-frames"
        and used by "external-calibration". Should be kept
        in-sync with output naming convention of upsp-extract-frames.
        """
        img_dir = _app_products_path(cfg, "first-frame", run_number)
        info = {"src_filename": [], "img_prefix": [], "img_filename": []}
        for camera_filename in _camera_filenames(cfg, run_number):
            camera_name = _camera_name(camera_filename)
            prefix = os.path.join(img_dir, camera_name)
            filename = "%s.%05d.%s" % (prefix, img_frame_number, img_ext)
            info["src_filename"].append(camera_filename)
            info["img_prefix"].append(prefix)
            info["img_filename"].append(filename)
        return info

    exe_sh_lines = [
        "# By default, core dumps are written out to the current working",
        "# directory of the process. We want core dumps to be written",
        "# to the process logs directory.",
        "cd %s" % _app_logs_path(cfg, step_name, run_number),
    ]

    info = _first_frames_info(cfg, run_number)
    for src_filename, img_prefix, img_filename in zip(
        info["src_filename"], info["img_prefix"], info["img_filename"]
    ):
        src_name = os.path.basename(src_filename)
        log_filename = _app_logs_path(cfg, step_name, run_number, "%s.out" % src_name)
        exe_sh_lines.extend(
            [
                "%s \\" % (shutil.which("upsp-extract-frames"),),
                "  -input=%s \\" % src_filename,
                "  -output=%s.%s \\" % (img_prefix, img_ext),
                "  -start=%d \\" % (img_frame_number),
                "  -count=1 \\",
                "  -dumpheader \\",  # log additional video file info
                "  > %s 2>&1\n" % log_filename,
            ]
        )
    exe_sh = "\n".join(exe_sh_lines)
    env_sh = _launcher_env_sh(cfg)
    exe_filename = _app_exec_path(cfg, step_name, run_number, _DEFAULT_TASK_EXE_NAME)
    _create_dir_and_log(_app_logs_path(cfg, step_name, run_number))
    _create_dir_and_log(_app_exec_path(cfg, step_name, run_number))
    for img_filename in info["img_filename"]:
        _create_dir_and_log(os.path.dirname(img_filename))
    _create_launcher(exe_filename, exe_sh=exe_sh, env_sh=env_sh)
    return info, [_DEFAULT_TASK_EXE_NAME]


def _create_task_external_calibration(
    cfg: dict, step_name: str, run_number: str, first_frames_info: dict
):
    this_run = cfg["datapoints"][run_number]
    external_cal_cfg_filename = _app_exec_path(cfg, step_name, run_number, "cfg.json")
    exe_sh_lines = []
    log_filename = _app_logs_path(cfg, step_name, run_number, "%s.out" % run_number)
    out_dir = _app_products_path(cfg, step_name, run_number)
    img_lines = ["  --img %s \\" % f for f in first_frames_info["img_filename"]]
    exe_sh_lines.extend(
        [
            "# By default, core dumps are written out to the current working",
            "# directory of the process. We want core dumps to be written",
            "# to the process logs directory.",
            "cd %s" % _app_logs_path(cfg, step_name, run_number),
            "%s \\" % (shutil.which("upsp-external-calibration"),),
            "  --tgts %s \\" % this_run["targets_file"],
            "  --grd %s \\" % this_run["grid_file"],
            "  --wtd %s \\" % this_run["wtd_file"],
            "  --cfg %s \\" % external_cal_cfg_filename,
            "  --cal_dir %s \\" % this_run["camera_tunnel_calibration_dir"],
            "  --out_dir %s \\" % out_dir,
        ]
    )
    exe_sh_lines.extend(img_lines)
    exe_sh_lines.extend(["  > %s 2>&1\n" % log_filename])
    _create_dir_and_log(os.path.dirname(log_filename))
    _create_dir_and_log(out_dir)
    exe_sh = "\n".join(exe_sh_lines)
    env_sh = _launcher_env_sh(cfg)
    exe_filename = _app_exec_path(cfg, step_name, run_number, _DEFAULT_TASK_EXE_NAME)
    _create_dir_and_log(os.path.dirname(exe_filename))
    _create_launcher(exe_filename, exe_sh=exe_sh, env_sh=env_sh)
    pcfg = _datapoint_processing_config(cfg, "external-calibration", run_number)
    io.json_write_or_die(pcfg, external_cal_cfg_filename, indent=2)

    info = {"calibration_filename": []}
    for camera_filename in _camera_filenames(cfg, run_number):
        camera_name = _camera_name(camera_filename)
        filename = os.path.join(out_dir, "%s-to-model.json" % camera_name)
        info["calibration_filename"].append(filename)
    return info, [_DEFAULT_TASK_EXE_NAME]


# cfg: dict, step_name: str, run_number: str, first_frames_info: dict
def _create_task_psp_process(
    cfg: dict, step_name: str, run_number: str, external_calibration_info: dict
):
    _create_input_file(cfg, step_name, run_number, external_calibration_info)
    _create_pbs_file(cfg, step_name, run_number, "run-psp-process.pbs")
    _create_add_field_file(cfg, step_name, run_number, "run-add-field.sh")
    return None, ["run-psp-process.pbs", "run-add-field.sh"]


def _create_per_datapoint_step(cfg: dict, step_name: str, fn, inputs=None):
    output = {}
    for run_number, _ in cfg["datapoints"].items():
        args = inputs[run_number][0] if inputs else []
        kwargs = inputs[run_number][1] if inputs else {}
        output[run_number], exe_names = fn(cfg, step_name, run_number, *args, **kwargs)
    for name in exe_names:
        if name == _DEFAULT_TASK_EXE_NAME:
            _create_step_launcher(cfg, step_name)
        else:
            _create_step_launcher(cfg, step_name, exe_name=name)
    _create_dir_and_log(_app_logs_path(cfg, step_name, "jobs"))
    return output


def _create(cfg: dict):
    _create_root_dir(cfg)
    _create_dir_and_log(_configuration_path(cfg))
    _create_dir_and_log(_launchers_path(cfg))
    _create_dir_and_log(_processing_path(cfg))
    _create_dir_and_log(_products_path(cfg))

    first_frame_info = _create_per_datapoint_step(
        cfg, "extract-first-frame", _create_task_extract_first_frame
    )

    external_calibration_info = _create_per_datapoint_step(
        cfg,
        "external-calibration",
        _create_task_external_calibration,
        {dp: [[v], {}] for dp, v in first_frame_info.items()},
    )

    _create_per_datapoint_step(
        cfg,
        "psp_process",
        _create_task_psp_process,
        {dp: [[v], {}] for dp, v in external_calibration_info.items()},
    )

    # TODO: re-enable this task, which is a step to render views of the
    #       model colored by certain scalars output by psp_process (the
    #       demo application made use of the NAS-provided TecPlot install,
    #       but has not been made general enough to include in the batch
    #       processing autogeneration tools). We will likely want to rewrite
    #       the step to make use of the PyTecplot interface, which is a
    #       lightweight Python package that connects to a Tecplot backend
    #       provided it is running and then provides an API for plotting.
    # _create_per_datapoint_step(
    #     cfg,
    #     "render-images",
    #     _create_task_render_images,
    # )

    _create_qsub_step_launcher(cfg)

    return _root_path(cfg)
