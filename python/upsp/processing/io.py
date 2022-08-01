import json
import logging
import sys

log = logging.getLogger(__name__)


_CHMOD_UR__GR__O__ = 0o0440
_CHMOD_URW_GR__O__ = 0o0640
_CHMOD_UR_XGR_XO__ = 0o0550
_CHMOD_URWXGR_XO__ = 0o0750


def json_load_or_die(filename):
    with open(filename, "r") as fp:
        try:
            return json.load(fp)
        except json.JSONDecodeError as e:
            log.fatal("While loading %s: %s", filename, e)
            sys.exit(1)


def json_write_or_die(obj, filename, **kwargs):
    try:
        with open(filename, "w") as fp:
            json.dump(obj, fp, **kwargs)
    except Exception as e:
        log.fatal("While writing %s: %s", filename, str(e))
        sys.exit(1)


def user_confirm_or_exit(question, cleanup_fn=None):
    check = str(input(question + " (y/n): ")).lower().strip()
    try:
        if check == "y":
            return True
        elif check == "n":
            print("Exiting.")
            if cleanup_fn is not None:
                cleanup_fn()
            sys.exit(0)
        else:
            print('Please enter "y" or "n"')
            return user_confirm_or_exit(question)
    except Exception as error:
        print(error)
        print('Please enter "y" or "n"')
        return user_confirm_or_exit(question)
