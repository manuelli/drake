__author__ = 'manuelli'
import os

def main():

    drake_source_dir = os.getenv('DRAKE_SOURCE_DIR')

    if drake_source_dir is None:
        raise EnvironmentError('The $DRAKE_SOURCE_DIR variable must be set to the drake source directory')
        return

    # director_config_filename = 'drake/examples/kuka_iiwa_arm/director_config.json'
    director_kuka_command = "directorPython $DRAKE_SOURCE_DIR/drake/examples/kuka_iiwa_arm/director_ik_app.py" \
                            " --director_config $DRAKE_SOURCE_DIR/drake/examples/kuka_iiwa_arm/director_config.json"
    startup_command = " --startup $DRAKE_SOURCE_DIR/drake/examples/ContactParticleFilter/src/runContactParticleFilterStartup.py"


    shell_command = director_kuka_command + startup_command
    print shell_command
    os.system(shell_command)


if __name__ == '__main__':
    main()