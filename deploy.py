

import sys
import subprocess

def generate_python(input_file):
    output_file = "CyPyHous3/src/apps/tasks.py"

    koord_exec = "koord/target/koord-1.0-SNAPSHOT-jar-with-dependencies.jar"


    subprocess.run(["java", "-jar", koord_exec, input_file, output_file])


generate_python(sys.argv[1])
