"""
get_build_flags.py
"""

import datetime
import os
import sys

utc_now = datetime.datetime.now()
compilation_date = utc_now.strftime("%Y-%m-%d")
compilation_time = utc_now.strftime("%H:%M:%S")
project_path = os.getcwd()
python_version = (
    f"{sys.version_info.major}"
    f".{sys.version_info.minor}"
    f".{sys.version_info.micro}"
)
python_path = os.path.realpath(sys.executable)
flags = f"""
    -D COMPILATION_DATE=\\"{compilation_date}\\"
    -D COMPILATION_TIME=\\"{compilation_time}\\"
    -D PROJECT_PATH=\\"{project_path}\\"
    -D PYTHON_VERSION=\\"{python_version}\\"
    -D PYTHON_PATH=\\"{python_path}\\"
"""

print(flags)
