__author__ = 'manuelli'
import os

def main():
    # director_config_filename = 'drake/examples/kuka_iiwa_arm/director_config.json'
    director_kuka_command = "directorPython $DRAKE_SOURCE_DIR/drake/examples/kuka_iiwa_arm/director_ik_app.py" \
                            " --director_config $DRAKE_SOURCE_DIR/drake/examples/kuka_iiwa_arm/director_config.json"
    startup_command = " --startup $DRAKE_SOURCE_DIR/drake/examples/ContactParticleFilter/src/runContactParticleFilterStartup.py"


    shell_command = director_kuka_command + startup_command
    print shell_command
    os.system(shell_command)
    # subprocess.check_call(shlex.split(shell_command))

    print "executed subprocess call"


if __name__ == '__main__':
    main()